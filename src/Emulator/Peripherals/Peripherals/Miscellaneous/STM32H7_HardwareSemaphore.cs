//
// Copyright (c) 2010-2025 Antmicro
//
// This file is licensed under the MIT License.
// Full license text is available in 'licenses/MIT.txt'.
//

using Antmicro.Renode.Core;
using Antmicro.Renode.Core.Structure.Registers;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals.Bus;

namespace Antmicro.Renode.Peripherals.Miscellaneous
{
    [AllowedTranslations(AllowedTranslation.ByteToDoubleWord | AllowedTranslation.WordToDoubleWord)]
    public sealed class STM32H7_HardwareSemaphore : BasicDoubleWordPeripheral, IKnownSize
    {
        public STM32H7_HardwareSemaphore(IMachine machine) : base(machine)
        {
            this.IRQ = new GPIO();
            this.IRQ2 = new GPIO();

            for(var i = 0; i < SemaphoreCount; ++i)
            {
                semaphores[i] = new Semaphore();
            }

            DefineRegisters();
        }

        public override void Reset()
        {
            lock(lockObject)
            {
                base.Reset();
                foreach(var semaphore in semaphores)
                {
                    semaphore.Reset();
                }
                c1InterruptStatus = 0;
                c2InterruptStatus = 0;
                IRQ.Unset();
                IRQ2.Unset();
            }
        }

        public override uint ReadDoubleWord(long offset)
        {
            lock(lockObject)
            {
                return RegistersCollection.Read(offset);
            }
        }

        public override void WriteDoubleWord(long offset, uint value)
        {
            lock(lockObject)
            {
                RegistersCollection.Write(offset, value);
            }
        }

        public GPIO IRQ { get; }

        // Second interrupt line, routed to the Cortex-M4 NVIC. The original
        // upstream model only exposed the C1 line; dual-core boot (CM4 parked
        // in STOP until CM7 releases HSEM_ID_0) needs the C2 notification to
        // actually wake the second core.
        public GPIO IRQ2 { get; }

        public long Size => 0x400;

        private void DefineRegisters()
        {
            //  Two-step lock registers (HSEM_Rx). Writing LOCK=1 with
            //  identifiers attempts a lock; writing the matching identifiers
            //  with LOCK=0 (what HAL_HSEM_Release does) releases.
            Registers.Semaphore.DefineMany(this, SemaphoreCount, (reg, idx) =>
            {
                reg
                    .WithValueField(0, 8, out var processIdBits, name: "PROCID", valueProviderCallback: _ => semaphores[idx].ProcessID)
                    .WithValueField(8, 8, out var masterIdBits, name: "MASTERID", valueProviderCallback: _ => semaphores[idx].MasterID)
                    .WithReservedBits(16, 15)
                    .WithFlag(31, out var lockBits, name: "LOCK", valueProviderCallback: _ => semaphores[idx].Locked)
                    .WithWriteCallback((_, __) =>
                    {
                        if(semaphores[idx].WriteLock(lockBits.Value, (uint)processIdBits.Value, (uint)masterIdBits.Value))
                        {
                            // The write released the semaphore: flag both
                            // cores and raise whichever interrupt line is
                            // enabled.
                            OnSemaphoreReleased(idx);
                        }
                    });
            });

            //  One-step lock, read-only
            //  Locking happens automatically just by reading the register
            Registers.ReadLockSemaphore.DefineMany(this, SemaphoreCount, (reg, idx) =>
            {
                reg
                    .WithValueField(0, 8, FieldMode.Read, name: "PROCID", valueProviderCallback: _ => semaphores[idx].ProcessID)
                    .WithValueField(8, 8, FieldMode.Read, name: "MASTERID", valueProviderCallback: _ => semaphores[idx].MasterID)
                    .WithReservedBits(16, 15)
                    .WithFlag(31, FieldMode.Read, name: "LOCK", valueProviderCallback: _ => semaphores[idx].Locked)
                    .WithReadCallback((_, __) =>
                    {
                        // When software uses 1 Step locking method, hardware is expected
                        // to read MasterID from AHB bus, which is a constant value dependent
                        // on the CPU type. There are 2 possible values: 0x3 for Cortex-M7
                        // and 0x1 for Cortex-M4
                        if(!machine.SystemBus.TryGetCurrentCPU(out var cpu))
                        {
                            this.Log(LogLevel.Warning, "Failed getting current CPU");
                            return;
                        }

                        var masterId = 0u;
                        if(cpu.Model == "cortex-m4" || cpu.Model == "cortex-m4f")
                        {
                            masterId = 0x1;
                        }
                        else if(cpu.Model == "cortex-m7")
                        {
                            masterId = 0x3;
                        }
                        else
                        {
                            this.Log(LogLevel.Warning, "Unsupported cpu model: {0}", cpu.Model);
                            return;
                        }
                        semaphores[idx].ReadLock(masterId);
                    });
            });

            //  Interrupt enable/status/clear registers, one set per core
            //  (C1 for Cortex-M7, C2 for Cortex-M4). A release flags the
            //  semaphore in both cores' status registers; each core's IRQ
            //  line asserts only while it has an enabled pending flag.
            //  Note the status bits are also set when the matching enable is
            //  clear, so enabling notifications after a release (like the
            //  dual-core boot sequence does) still raises the interrupt.
            Registers.InterruptEnableC1.Define(this)
                .WithValueField(0, 32, out c1InterruptEnable, name: "ISE",
                    writeCallback: (_, __) => UpdateInterrupts());
            Registers.InterruptEnableC2.Define(this)
                .WithValueField(0, 32, out c2InterruptEnable, name: "ISE",
                    writeCallback: (_, __) => UpdateInterrupts());
            Registers.InterruptClearC1.Define(this)
                .WithValueField(0, 32, name: "IC",
                    writeCallback: (_, value) => { c1InterruptStatus &= (uint)~value; UpdateInterrupts(); });
            Registers.InterruptClearC2.Define(this)
                .WithValueField(0, 32, name: "IC",
                    writeCallback: (_, value) => { c2InterruptStatus &= (uint)~value; UpdateInterrupts(); });
            Registers.InterruptStatusC1.Define(this)
                .WithValueField(0, 32, FieldMode.Read, name: "ISF",
                    valueProviderCallback: _ => c1InterruptStatus);
            Registers.InterruptStatusC2.Define(this)
                .WithValueField(0, 32, FieldMode.Read, name: "ISF",
                    valueProviderCallback: _ => c2InterruptStatus);
            Registers.MaskedInterruptStatusC1.Define(this)
                .WithValueField(0, 32, FieldMode.Read, name: "MISF",
                    valueProviderCallback: _ => c1InterruptStatus & c1InterruptEnable.Value);
            Registers.MaskedInterruptStatusC2.Define(this)
                .WithValueField(0, 32, FieldMode.Read, name: "MISF",
                    valueProviderCallback: _ => c2InterruptStatus & c2InterruptEnable.Value);
        }

        private void OnSemaphoreReleased(int index)
        {
            var mask = 1u << index;
            c1InterruptStatus |= mask;
            c2InterruptStatus |= mask;
            UpdateInterrupts();
        }

        private void UpdateInterrupts()
        {
            if((c1InterruptStatus & c1InterruptEnable.Value) != 0)
            {
                IRQ.Set();
            }
            else
            {
                IRQ.Unset();
            }
            if((c2InterruptStatus & c2InterruptEnable.Value) != 0)
            {
                IRQ2.Set();
            }
            else
            {
                IRQ2.Unset();
            }
        }

        private readonly object lockObject = new object();
        private readonly Semaphore[] semaphores = new Semaphore[SemaphoreCount];

        private uint c1InterruptStatus;
        private uint c2InterruptStatus;
        private IValueRegisterField c1InterruptEnable;
        private IValueRegisterField c2InterruptEnable;

        private const uint SemaphoreCount = 32;

        private class Semaphore
        {
            public Semaphore()
            {
                Reset();
            }

            //  Reset the state of the semaphore
            public void Reset()
            {
                Locked = false;
                ProcessID = 0;
                MasterID = 0;
            }

            //  Attempt to lock the semaphore via 1-step read lock
            public void ReadLock(uint masterId)
            {
                //  Process ID is unused and is always 0 in this case
                TryLock(0x0000, masterId);
            }

            //  Handles a write directed at the 2-step write lock register.
            //  Returns true when the write released a previously locked
            //  semaphore. Matches the hardware: LOCK=1 attempts a lock,
            //  LOCK=0 with matching identifiers releases.
            public bool WriteLock(bool lockBit, uint processId, uint masterId)
            {
                if(lockBit)
                {
                    TryLock(processId, masterId);
                    return false;
                }
                else
                {
                    return TryUnlock(processId, masterId);
                }
            }

            public bool Locked { get; private set; }

            public uint ProcessID { get; private set; }

            public uint MasterID { get; private set; }

            //  Tries to acquire a lock on the semaphore, with the given process ID and master ID
            private void TryLock(uint processId, uint masterId)
            {
                if(Locked)
                {
                    return;
                }

                Locked = true;
                ProcessID = processId;
                MasterID = masterId;
            }

            //  Tries to unlock the semaphore with the given credentials.
            //  The semaphore will only be unlocked if it was previously locked,
            //  and its process ID and the master ID match the ones used for
            //  unlocking. Returns true when the semaphore was released.
            private bool TryUnlock(uint processId, uint masterId)
            {
                if(!Locked)
                {
                    return false;
                }
                if(ProcessID != processId)
                {
                    return false;
                }
                if(MasterID != masterId)
                {
                    return false;
                }

                Reset();
                return true;
            }
        }

        private enum Registers
        {
            Semaphore = 0x0,
            //  The above serves as a base offset, 32 double word registers follow
            ReadLockSemaphore = 0x80,
            //  The above serves as a base offset, 32 double word registers follow
            InterruptEnableC1 = 0x100,
            InterruptClearC1 = 0x104,
            InterruptStatusC1 = 0x108,
            MaskedInterruptStatusC1 = 0x10c,
            InterruptEnableC2 = 0x110,
            InterruptClearC2 = 0x114,
            InterruptStatusC2 = 0x118,
            MaskedInterruptStatusC2 = 0x11c,
            Clear = 0x140,
            Key = 0x144
        }
    }
}
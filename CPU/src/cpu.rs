// use crate::opcode;
// use std::collections::HashMap;

#[warn(non_snake_case)]

pub mod cpu {

    ///
    ///  7 6 5 4 3 2 1 0
    ///  N V _ B D I Z C
    ///  | |   | | | | +--- Carry Flag
    ///  | |   | | | +----- Zero Flag
    ///  | |   | | +------- Interrupt Disable
    ///  | |   | +--------- Decimal Mode (not used on NES)
    ///  | |   +----------- Break Command
    ///  | +--------------- Overflow Flag
    ///  +----------------- Negative Flag
    ///
    bitflags!{
        #[derive(PartialEq, Eq)]
        #[derive(Clone)]
        pub struct CPUflags:u8 {
            const CARRY = 0b0000_0001;
            const ZERO = 0b0000_0010;
            const INTERRUPT_DISABLE = 0b0000_0100;
            const DECIMAL_MODE = 0b0000_1000;
            const BREAK = 0b0001_0000;
            const BREAK2 = 0b0010_0000;
            const OVERFLOW = 0b0100_0000;
            const NEGATIVE = 0b1000_0000;
        }
    }
   
use crate::opcode::opcode;
#[derive(Debug)]
#[allow(non_camel_case_types)]
pub enum AddressModes {
    immediate,
    zero_page,
    zero_page_x,
    zero_page_y,
    absolute,
    absolute_x,
    absolute_y,
    Indexed_Indirect_x,
    Indirect_Indexed_y,
    NonaddressingMode,  

}

//Define stack memory address
const STACK_OFFSET: u16 = 0x0100;//Length of stack from 0x0100 to 0x01ff
const STACK_RESET: u8 = 0xfd; //this is the initial stack pointer. When a push is done, this pointer is added to stack_offset and data is stored at the address obtained from the addition 
// and this stack_reset is decremented. When a pop is done, the stack_reset is incremented and the data is read from the address obtained from the addition of stack_offset and stack_reset
#[warn(non_snake_case)]
pub trait Mem {
    //Traits' functions are inherently public
    //We can't implement this two traits because, they require access to to the memory of CPU...which varies from platform to platform
    fn mem_read(&self , address: u16) -> u8;
    fn mem_write(&mut self, address: u16, data: u8);
    
    //This we can implement cause it is independent of the platform, it is just bit arthimetic that leverages the above two functions
    fn mem_read_16(&self, address: u16) -> u16 {
        let lower_byte = self.mem_read(address) as u16;
        let higher_byte = self.mem_read(address.wrapping_add(1)) as u16;

        (higher_byte << 8) | lower_byte

    }

    fn mem_write_16(&mut self, data: u16, address:u16) {
        let higher_byte = (data >> 8) as u8;
        let lower_byte = (data & 0xff) as u8;
        self.mem_write(address, lower_byte);
        self.mem_write(address.wrapping_add(1), higher_byte);

    }
    
}
pub struct CPU {
    //Declaring general Purpose 8 bit register
    pub a_reg: u8,
    pub x_reg: u8,
    pub y_reg: u8,
    pub stack_pointer: u8,
    pub status_reg: CPUflags, //8 bit status_register
    pub pc: u16, //16 bit Program Counter Register. Why 16 bit? Cause address line is 16 bit
    memory: [u8;0xFFFF] //Creating a memory space of 65535
}

impl Mem for CPU {
    fn mem_read(&self, address:u16) -> u8 {
        self.memory[address as usize]
    }

    fn mem_write(&mut self, address: u16, data: u8) {
        self.memory[address as usize] = data;
    }
}

impl CPU {
    pub fn new() -> Self {
        Self {
            a_reg: 0,
            status_reg: CPUflags::from_bits_truncate(0b100_100), // what this does is, it sets the interrupt disable and break 2 flag to 1. So initially interrupt is disabled
            // and break 2 is enabled. 
            pc: 0,
            x_reg: 0,
            y_reg: 0,
            stack_pointer: STACK_RESET,
            memory: [0;0xFFFF]
        }
    }
    //NES platform has a special mechanism to mark where the CPU should start the execution. 
    //Upon inserting a new cartridge, the CPU receives a special signal called "Reset interrupt" that instructs CPU to:
    // --> Reset the state(register and flags)
    // --> set the program counter to memory address stored at memory location 0xfffc

    pub fn load_and_run(&mut self, program:Vec<u8>) {
        self.load(program);
        self.reset();
        self.run()
    }

    pub fn load(&mut self, program:Vec<u8>) {
        self.memory[0x8000 .. (0x8000 + program.len())].copy_from_slice(&program);
        // The program begins from 0x8000, so at memory location 0xfffc(address) the data 0x8000(address) should be stored. That is:
        //16 bit address must be written at memory location 0xfffc
        self.mem_write_16(0x8000, 0xfffc);
    }

    pub fn reset(&mut self) {
        self.a_reg = 0;
        self.x_reg =  0;
        self.y_reg = 0;
        self.status_reg = CPUflags::from_bits_truncate(0b100_100);
        self.stack_pointer = STACK_RESET; 
        self.pc = self.mem_read_16(0xfffc);
    }

    pub fn run(&mut self) {
        let ref  opcodes = *opcode::OPCODES_MAP;
    
        loop {
            let code = self.mem_read(self.pc); 
            self.pc += 1;
            let program_counter_state = self.pc;
            let opcode = opcodes.get(&code).expect(&format!("OpCode {:x} is not recognized",code));

            match code {
                //Group this under one mnemonic for each opcodes
                /*LDA */
                0xA9 | 0xA5 | 0xB5 | 0xad | 0xbd | 0xb9 | 0xa1 | 0xb1 => {
                    self.lda(&opcode.mode);
                },
                /*ADC */
                0x69 | 0x65 | 0x75 | 0x6d  | 0x7d | 0x79 | 0x61 | 0x71 => {
                    self.adc(&opcode.mode);
                },
                /*AND */
                0x29 | 0x25 | 0x35 | 0x2d | 0x3d | 0x39 | 0x21 | 0x31 => {
                    self.and(&opcode.mode);
                },
                 /* ASL */
                0x06 | 0x16 | 0x0e | 0x1e => {
                    self.asl(&opcode.mode);
                },
                /*ASL_ACCUMULATOR */
                0x0a => {
                    self.asl_accumulator();
                },
                /*BCC */
                0x90 => {
                    self.branch(!self.status_reg.contains(CPUflags::CARRY));
                },
                /*BCS */
                0xb0 => {
                    self.branch(self.status_reg.contains(CPUflags::CARRY));
                },
                /*BEQ */
                0xf0 => {
                    self.branch(self.status_reg.contains(CPUflags::ZERO));
                },

                /*BIT */
                0x24 | 0x2c => {
                    self.bit(&opcode.mode);
                },

                /*BMI */
                0x30 => {
                    self.branch(self.status_reg.contains(CPUflags::NEGATIVE));
                },

                /*BNE */
                0xd0 => {
                    self.branch(!self.status_reg.contains(CPUflags::ZERO));
                },

                /*BPL */
                0x10 => {
                    self.branch(!self.status_reg.contains(CPUflags::NEGATIVE));
                },

                /*BRK */
                0x00 => {
                    return;
                },

                /*BVC */
                0x50 => {
                    self.branch(!self.status_reg.contains(CPUflags::OVERFLOW));
                },

                /*BVS */
                0x70 => {
                    self.branch(self.status_reg.contains(CPUflags::OVERFLOW));
                },
                /*CLC */
                0x18 => {
                    self.clear_flag(&CPUflags::CARRY);
                },
                /*CLD */
                0xd8 => {
                    self.clear_flag(&CPUflags::DECIMAL_MODE);
                },
                /*CLI */
                0x58 => {
                    self.clear_flag(&CPUflags::INTERRUPT_DISABLE);
                },
                /*CLV*/
                0xb8 => {
                    self.clear_flag(&CPUflags::OVERFLOW);
                },
                /*CMP */
                0xc9 | 0xc5 | 0xd5 | 0xcd | 0xdd | 0xd9 | 0xc1 | 0xd1 => {
                    self.compare(&opcode.mode, self.a_reg);
                },
                /*CPX */
                0xe0 | 0xe4 | 0xec => {
                    self.compare(&opcode.mode, self.x_reg);
                },
                /*CPY */
                0xc0 | 0xc4 | 0xcc => {
                    self.compare(&opcode.mode, self.y_reg);
                },

                /*DEC */
                0xc6 | 0xd6 | 0xce | 0xde => {
                    self.dec(&opcode.mode);
                },
                
                /*DEX */
                0xca => {
                    self.dex();
                },
                /*DEY */
                0x88 => {
                    self.dey();
                },
                /*EOR */
                0x49 | 0x45 | 0x55 | 0x4d | 0x5d | 0x59 | 0x41 | 0x51 => {
                    self.eor(&opcode.mode);
                },
                /*INC */
                0xe6 | 0xf6 | 0xee | 0xfe => {
                    self.inc(&opcode.mode);
                },
                /*INX */
                0xe8 => {
                    self.inx();
                },
                /*INY */
                0xc8 => {
                    self.iny();
                },
                /*JMP_Absolute */
                0x4c => {
                    let address = self.mem_read_16(self.pc);
                    self.pc = address;
                },
                /*JMP_INDIRECT */
                0x6c => {
                    let address = self.mem_read_16(self.pc);
                    //let address = $30FF 
                    //$30FF <- $80
                    // $3100 <- $50
                    //$3000 <- 40
                    //the control will be passed to 4080 rather than, 5080...that is the lower byte of the address wrapped.
                    //This is called page boundary in jumping

                    let indirect_ref = if address & 0x00FF == 0x00FF {
                        let lo = self.mem_read(address);
                        let hi = self.mem_read(address & 0xFF00);
                        (hi as u16) << 8 | (lo as u16)
                    }
                    else {
                        self.mem_read_16(address)
                    };

                    self.pc = indirect_ref;
                },
                /*JSR */
                0x20 => {
                    self.stack_push_16(self.pc + 2 - 1);
                    let target_address = self.mem_read_16(self.pc);
                    self.pc = target_address;
                },
                /*RTS */
                0x60 => {
                    self.pc = self.stack_pop_16() + 1;
                },

                /*RTI */
                0x40 => {
                    self.status_reg = CPUflags::from_bits_truncate(self.stack_pop());
                    self.status_reg.remove(CPUflags::BREAK);
                    self.status_reg.insert(CPUflags::BREAK2);

                    self.pc = self.stack_pop_16();
                },

                /*LDX */
                0xa2 | 0xa6 | 0xb6 | 0xae | 0xbe => {
                    self.ldx(&opcode.mode);
                },

                /*LDY */
                0xa0 | 0xa4 | 0xb4 | 0xac | 0xbc => {
                    self.ldy(&opcode.mode);
                },

                /*LSR_Accumulator */
                0x4a => {
                    self.lsr_accumulator();
                },

                /*LSR */
                0x46 | 0x56 | 0x4e | 0x5e => {
                    self.lsr(&opcode.mode);
                },

                /*NOP */
                0xea => {
                    //Do nothing
                },

                /*ORA */
                0x09 | 0x05 | 0x15 | 0x0d | 0x1d | 0x19 | 0x01 | 0x11 => {
                    self.ora(&opcode.mode);
                },
                /*PHA */
                0x48 => {
                    self.stack_push(self.a_reg);
                },
                /*PHP */
                0x08 => {
                    self.php();
                },
                /*pla */
                0x68 => {
                    self.pla();
                },
                /*plp */
                0x28 => {
                    self.plp();
                },
                /*ROL */
                0x26 | 0x36 | 0x2e | 0x3e => {
                    self.rol(&opcode.mode);
                },
                /*ROL_ACCUMULATO */
                0x2a => {
                    self.rol_accumulator();
                },
                /*ROR*/
                0x66 | 0x76 | 0x6e | 0x7e => {
                    self.ror(&opcode.mode);
                },
                /*ROR_Accumulator */
                0x6a => {
                    self.ror_accumulator();
                },
                /*sbc */
                0xe9 | 0xe5 | 0xf5 | 0xed | 0xfd | 0xf9 | 0xe1 | 0xf1 => {
                    self.sbc(&opcode.mode);
                },
                /*sec */
                0x38 => {
                    self.set_flag(&CPUflags::CARRY);
                },
                /*SED */
                0xf8 => {
                    self.set_flag(&CPUflags::DECIMAL_MODE);
                },
                /*SEI */
                0x78 => {
                    self.set_flag(&CPUflags::INTERRUPT_DISABLE);
                },
                /*STA */
                0x85 | 0x95 | 0x8d | 0x9d | 0x99 | 0x81 | 0x91 => {
                    self.sta(&opcode.mode);
                },
                /*STX */
                0x86 | 0x96 | 0x8e => {
                    self.stx(&opcode.mode);
                },
                /*sty */
                0x84 | 0x94 | 0x8c => {
                    self.sty(&opcode.mode);
                },
                /*tax */
                0xaa => {
                    self.tax();
                },
                /*tay */
                0xa8 => {
                    self.tay();
                },
                /*tsx */
                0xba => {
                    self.x_reg = self.stack_pointer;
                    self.update_zero_and_negative_flags(self.x_reg);
                },
                /*TXA */
                0x8a => {
                    self.set_register_a(self.x_reg);
                },
                /*TXS */
                0x9a => {
                    self.stack_pointer = self.x_reg;
                    self.update_zero_and_negative_flags(self.stack_pointer);
                },
                /*TYA */
                0x98 => {
                    self.set_register_a(self.y_reg);
                },
                _ => todo!(),            
        }
        if program_counter_state == self.pc {
            self.pc += (opcode.len - 1) as u16;
        } 
    }}




    fn update_zero_and_negative_flags(&mut self, current_result: u8) {
        //Setting the zero Flag
        if current_result == 0 {
                self.status_reg.insert(CPUflags::ZERO); // 0 flag MUST be set no matter what, and other flag unaffected, so OR operation is required
        }
        else {
            self.status_reg.remove(CPUflags::ZERO);
        }
        //setting the negative flag
        if current_result & 0b1000_0000 != 0 { // check the msb of the result
            self.status_reg.insert(CPUflags::NEGATIVE); // MSB = 1, set NegativeFlag = 1
        }
        else {
            self.status_reg.remove(CPUflags::NEGATIVE); // MSB = 0, set NegativeFlag = 0) ; 

        }
    }

    fn get_operand_address(&self, mode : &AddressModes) -> u16 {

        match mode {
            AddressModes::immediate => self.pc,
            AddressModes::zero_page => self.mem_read(self.pc) as u16,
            AddressModes::zero_page_x => {
                let pos = self.mem_read(self.pc);
                pos.wrapping_add(self.x_reg) as u16
            },
            AddressModes::zero_page_y => {
                let pos= self.mem_read(self.pc);
                pos.wrapping_add(self.y_reg) as u16
            }
            AddressModes::absolute => self.mem_read_16(self.pc),
            AddressModes::absolute_x => {
                let base_address = self.mem_read_16(self.pc);
                base_address.wrapping_add(self.x_reg as u16)
            },
            AddressModes::absolute_y => {
                let base_address = self.mem_read_16(self.pc);
                base_address.wrapping_add(self.y_reg as u16)
            },
            
            //This is a bit tricky.
            //                                                                        |self.pc is here|
            //What we are trying to do is LDA ($80,x); [opcodes] = [0xa9(lda opcode), 0x80(address), 0x00(next opcode)]
            //We ought to add 'x_register' to 80 say x is 2: so it becomes LDA ($82).... 
            // We need 2 byte address stored at 82 and 83 cause only one byte is stored at a memory cell:
            // lets say: |82| data: 00 (Lower Byte of address cause it is little endian)|
            //           |23| data: 20 (higher byte of the address)|
            // we simple do mem_read(82) (lowee byte) and mem_read(83) and combine the both using bit arthimetic to get the address of the operand


            AddressModes::Indexed_Indirect_x => {
                let base = self.mem_read(self.pc);
                let ptr = (base as u8).wrapping_add(self.x_reg) as u8;
                let lower_byte = self.mem_read(ptr as u16);
                let higher_byte = self.mem_read(ptr.wrapping_add(1) as u16);
                (higher_byte as u16) << 8 | (lower_byte as u16)
            },

            AddressModes::Indirect_Indexed_y => {
                let base = self.mem_read(self.pc);
                let lower_byte_address = self.mem_read(base as u16);
                let higher_byte_address = self.mem_read((base.wrapping_add(1)) as u16);
                let address = (higher_byte_address as u16) << 8 | (lower_byte_address as u16);
                address.wrapping_add(self.y_reg as u16)
            },
            AddressModes::NonaddressingMode => {
                panic!("Mode {:?} is not supported",mode)
            }

        }

    }

    //Instruction Set
    // Arthimetic and Logical Instructions
    fn set_register_a(&mut self, value: u8) {
        self.a_reg = value;
        self.update_zero_and_negative_flags(self.a_reg);
    }

    fn add_to_register_a(&mut self, data:u8) {
        //add, check carry, check overflow
        let temp = (self.a_reg as u16) + (data as u16) + (if self.status_reg.contains(CPUflags::CARRY) {1} else {0}) as u16;
        self.status_reg.set(CPUflags::CARRY, temp > 0xff);
        self.status_reg.set(CPUflags::OVERFLOW,(self.a_reg ^ data) & 0x80 == 0 && (self.a_reg ^ temp as u8) & 0x80 != 0);
        //                                      {if a_reg and data have same sign and a_reg and temp have different sign, then overflow}
        self.set_register_a(temp as u8); 

    }

    fn adc(&mut self, addressing_mode: &AddressModes) {
        self.add_to_register_a(self.mem_read(self.get_operand_address(addressing_mode)));
    }

    fn sbc(&mut self, addressing_mode:&AddressModes) {
        let address_operand = self.get_operand_address(addressing_mode);
        let data = self.mem_read(address_operand);
        self.add_to_register_a(((data as i8).wrapping_neg()).wrapping_sub(1) as u8);
        
    }

    fn and(&mut self,addressing_mode:&AddressModes) {
        self.set_register_a(self.a_reg & self.mem_read(self.get_operand_address(addressing_mode)));
    }

    fn eor(&mut self, addressing_mode: &AddressModes) {
        self.set_register_a(self.a_reg ^ self.mem_read(self.get_operand_address(addressing_mode)));
    }

    fn ora(&mut self, addressing_mode: &AddressModes) {
        self.set_register_a(self.a_reg | self.mem_read(self.get_operand_address(addressing_mode)));
    }

    fn inx(&mut self) {
        self.x_reg = self.x_reg.wrapping_add(1);
        self.update_zero_and_negative_flags(self.x_reg);
    }

    fn iny(&mut self) {
        self.y_reg = self.y_reg.wrapping_add(1);
        self.update_zero_and_negative_flags(self.y_reg);
    }

    fn inc(&mut self, addressing_mode:&AddressModes) {
        let operand_address = self.get_operand_address(addressing_mode);
        let mut data = self.mem_read(operand_address);
        data = data.wrapping_add(1);
        self.mem_write(operand_address, data);
        self.update_zero_and_negative_flags(data);
    }

    fn dex(&mut self) {
        self.x_reg = self.x_reg.wrapping_sub(1);
        self.update_zero_and_negative_flags(self.x_reg);
    }

    fn dey(&mut self) {
        self.y_reg = self.y_reg.wrapping_sub(1);
        self.update_zero_and_negative_flags(self.y_reg);
    }

    fn dec(&mut self, addressing_mode:&AddressModes) {
        let operand_address = self.get_operand_address(addressing_mode);
        let mut data = self.mem_read(operand_address);
        data = data.wrapping_sub(1);
        self.mem_write(operand_address, data);
        self.update_zero_and_negative_flags(data);
    }

    fn compare(&mut self, addressing_mode:&AddressModes, compare_with: u8) {
        let operand_address = self.get_operand_address(addressing_mode);
        let data = self.mem_read(operand_address);
        if compare_with == data {
            self.status_reg.insert(CPUflags::ZERO);
        }
        else if compare_with > data {
            self.status_reg.insert(CPUflags::CARRY);
        }
        else {
            self.status_reg.insert(CPUflags::NEGATIVE);
        }
    }
    //Loading and storing to/from memeory
    fn lda(&mut self , addressing_mode: &AddressModes) {
        let operand_address = self.get_operand_address(addressing_mode);
        self.set_register_a(self.mem_read(operand_address));
        }
    
    fn tax(&mut self) {
        self.x_reg = self.a_reg;
        self.update_zero_and_negative_flags(self.x_reg);
    }

    fn tay(&mut self) {
        self.y_reg = self.a_reg;
        self.update_zero_and_negative_flags(self.y_reg);
    }


    fn ldx(&mut self, addressing_mode: &AddressModes) {
        let operand_address =self.get_operand_address(addressing_mode);
        self.x_reg = self.mem_read(operand_address);
        self.update_zero_and_negative_flags(self.x_reg);
    } 
    
    fn ldy(&mut self, addressing_mode: &AddressModes) {
        let operand_address = self.get_operand_address(addressing_mode);
        self.y_reg = self.mem_read(operand_address);
        self.update_zero_and_negative_flags(self.y_reg);
    }

    fn sta(&mut self, addressing_mode:&AddressModes) {
        let address = self.get_operand_address(addressing_mode);
        self.mem_write(address, self.a_reg);
    }

    fn sty(&mut self, addressing_mode:&AddressModes) {
        let address = self.get_operand_address(addressing_mode);
        self.mem_write(address,self.y_reg);
    }

    fn stx(&mut self,addressing_mode: &AddressModes) {
        let address = self.get_operand_address(addressing_mode);
        self.mem_write(address,self.x_reg);
    }

    //Stack Instructions: all stack instructions are implied so we directly implement these instruction while decoding the opcode
    fn stack_push(&mut self, data:u8) {
        self.mem_write((STACK_OFFSET as u16) + self.stack_pointer as u16 , data);
        self.stack_pointer = self.stack_pointer.wrapping_sub(1);
    }

    fn stack_pop(&mut self) -> u8 {
        self.stack_pointer = self.stack_pointer.wrapping_sub(1);
        self.mem_read((STACK_OFFSET as u16) + self.stack_pointer as u16)
    }

    fn stack_push_16(&mut self, data: u16) {
        let higher_byte = (data >> 8) as u8;
        let lower_byte = (data & 0xff) as u8;
        self.stack_push(higher_byte);
        self.stack_push(lower_byte);
    }

    fn stack_pop_16(&mut self) -> u16 {
        let lower_byte = self.stack_pop();
        let higher_byte = self.stack_pop();
        (higher_byte as u16) << 8 | (lower_byte as u16)
    }

    fn php(&mut self) {
        let mut status_register = self.status_reg.clone();
        status_register.insert(CPUflags::BREAK);
        status_register.insert(CPUflags::BREAK2);
        self.stack_push(status_register.bits());
        
    }

    fn pla(&mut self) {
        let data = self.stack_pop();
        self.set_register_a(data);
    }

    fn plp(&mut self) {
        self.status_reg = CPUflags::from_bits(self.stack_pop()).unwrap();
        self.status_reg.remove(CPUflags::BREAK);
        self.status_reg.insert(CPUflags::BREAK2);
    }

    //Flags
    pub fn set_flag(&mut self, flag: &CPUflags) {
        match *flag {
            CPUflags:: CARRY => self.status_reg.insert(CPUflags::CARRY),
            CPUflags:: ZERO => self.status_reg.insert(CPUflags::ZERO),
            CPUflags:: INTERRUPT_DISABLE => self.status_reg.insert(CPUflags::INTERRUPT_DISABLE),
            CPUflags:: DECIMAL_MODE => self.status_reg.insert(CPUflags::DECIMAL_MODE),
            CPUflags:: BREAK => self.status_reg.insert(CPUflags::BREAK),
            CPUflags:: BREAK2 => self.status_reg.insert(CPUflags::BREAK2),
            CPUflags:: OVERFLOW => self.status_reg.insert(CPUflags::OVERFLOW),
            CPUflags:: NEGATIVE => self.status_reg.insert(CPUflags::NEGATIVE),
            _ => panic!("Flag not supported")
        }
    }

    pub fn clear_flag(&mut self, flag: &CPUflags) {
        match *flag {
            CPUflags:: CARRY => self.status_reg.remove(CPUflags::CARRY),
            CPUflags:: ZERO => self.status_reg.remove(CPUflags::ZERO),
            CPUflags:: INTERRUPT_DISABLE => self.status_reg.remove(CPUflags::INTERRUPT_DISABLE),
            CPUflags:: DECIMAL_MODE => self.status_reg.remove(CPUflags::DECIMAL_MODE),
            CPUflags:: BREAK => self.status_reg.remove(CPUflags::BREAK),
            CPUflags:: BREAK2 => self.status_reg.remove(CPUflags::BREAK2),
            CPUflags:: OVERFLOW => self.status_reg.remove(CPUflags::OVERFLOW),
            CPUflags:: NEGATIVE => self.status_reg.remove(CPUflags::NEGATIVE),
            _ => panic!("Flag not supported")
        }
    }

    //Shift and Rotate Instructions
    fn asl_accumulator(&mut self) {
        let mut data = self.a_reg;

        if data & 0x80 == 0x00 {
            self.clear_flag(&CPUflags::CARRY);
        }
        else {self.set_flag(&CPUflags::CARRY);}

        self.set_register_a(data << 1);
    }

    fn asl(&mut self, addressing_mode:&AddressModes) -> u8 {
        let operand_address = self.get_operand_address(addressing_mode);
        let mut data = self.mem_read(operand_address);
        if data & 0x80 == 0x00 {
            self.clear_flag(&CPUflags::CARRY);
        }
        else {
            self.set_flag(&CPUflags::CARRY);
        }
        data = data << 1;
        self.mem_write(operand_address,data);
        self.update_zero_and_negative_flags(data);
        data
    }

    fn lsr_accumulator(&mut self) {
        let mut data = self.a_reg;
        if data & 1 == 1 {
            self.set_flag(&CPUflags::CARRY);
        }
        else {
            self.clear_flag(&CPUflags::CARRY);
        }

        data = data >> 1;
        self.set_register_a(data);
    }

    fn lsr(&mut self, addressing_mode:&AddressModes) -> u8 {
        let operand_address = self.get_operand_address(addressing_mode);
        let mut data = self.mem_read(operand_address);
        if data & 1 == 1 {
            self.set_flag(&CPUflags::CARRY);
        }
        else {
            self.clear_flag(&CPUflags::CARRY);
        }

        data = data >> 1;
        self.mem_write(operand_address,data);
        self.update_zero_and_negative_flags(data);
        data
    }

    fn ror_accumulator(&mut self) {
        let mut data = self.a_reg;
        let old_carry = self.status_reg.contains(CPUflags::CARRY);

        if data & 1 == 1 {
            self.set_flag(&CPUflags::CARRY);
        }
        else {
            self.clear_flag(&CPUflags::CARRY);
        }

        data = data >> 1;

        if old_carry {
            data = data | 0x80;
        }
        else {
            data = data & 0b0111_1111;
        }
        self.set_register_a(data);
    }

    fn ror(&mut self, addressing_mode:&AddressModes) {
        let operand_address = self.get_operand_address(addressing_mode);
        let mut data = self.mem_read(operand_address);
        let old_carry = self.status_reg.contains(CPUflags::CARRY);

        if data & 1 == 1 {
            self.set_flag(&CPUflags::CARRY);
        }
        else {
            self.clear_flag(&CPUflags::CARRY);
        }

        data = data >> 1;

        if old_carry {
            data = data | 0x80;
        }
        else {
            data = data & 0b0111_1111;
        }
        self.mem_write(operand_address,data);
        self.update_zero_and_negative_flags(data);
    }

    fn rol_accumulator(&mut self) {
        let mut data = self.a_reg;
        let old_carry = self.status_reg.contains(CPUflags::CARRY);

        if data & 0x80 == 0 {
            self.clear_flag(&CPUflags::CARRY);
        }
        else {
            self.set_flag(&CPUflags::CARRY);
        }
        data = data << 1;
        if old_carry {
            data = data | 1;
        }
        else {
            data = data & 0b1111_1110;
        }
        self.set_register_a(data);

    }
    fn rol(&mut self, addressing_mode:&AddressModes) {
        let operand_address = self.get_operand_address(addressing_mode);
        let mut data = self.mem_read(operand_address);
        let old_carry = self.status_reg.contains(CPUflags::CARRY);

        if data & 0x80 == 0 {
            self.clear_flag(&CPUflags::CARRY);
        }
        else {
            self.set_flag(&CPUflags::CARRY);
        }
        data = data << 1;
        if old_carry {
            data = data | 1;
        }
        else {
            data = data & 0b1111_1110;
        }
        self.mem_write(operand_address,data);
        self.update_zero_and_negative_flags(data);
    }


    //Branching Instructions
    fn branch(&mut self, condition:bool) {
        if condition {
            let jump = self.mem_read(self.pc) as i8;
            let jump_address = self.pc.wrapping_add(1).wrapping_add(jump as u16);
            self.pc = jump_address;
        }
    }

    fn bit(&mut self, addressing_mode:&AddressModes) {
        let operand_address = self.get_operand_address(addressing_mode);

        let data = self.mem_read(operand_address);
        if data & self.a_reg == 0{
            self.set_flag(&CPUflags::ZERO);
        }
        else {
            self.clear_flag(&CPUflags::ZERO);
        }

        self.status_reg.set(CPUflags::NEGATIVE, data & 0b1000_0000 == 1);
        self.status_reg.set(CPUflags::OVERFLOW, data & 0b0100_0000 > 0);
    }
}
}

#[cfg(test)]
mod tests {
    use super::cpu::*;

    #[test]
    fn test_lda_immediate_load() {
        let mut cpu = CPU::new();
        let instructions = vec![0xa9,0x05, 0x00];
        cpu.load_and_run(instructions);

        assert_eq!(cpu.a_reg, 0x05);
        assert!(cpu.status_reg.contains(CPUflags::ZERO) == false);

    }

    #[test]
    fn test_0xa9_lda_zero_flag() {
        let mut cpu = CPU::new();
        cpu.load_and_run(vec![0xa9, 0x00, 0x00]);
        assert!(cpu.status_reg.contains(CPUflags::ZERO) == true);
    }
    #[test]
    fn test_0xa0_lda_negative_flag() {
        let mut cpu = CPU::new();
        cpu.load_and_run(vec![0xa9,0xff,0x00]);
        assert!(cpu.status_reg.contains(CPUflags::NEGATIVE) == true);
    }
    #[test]
    fn test_ops_working_together() {
        let mut cpu = CPU::new();
        cpu.load_and_run(vec![0xa9,0x05,0xaa,0xe8,0x00]);
        assert_eq!(cpu.x_reg, 0x06);
    }

    #[test]
    fn test_0xaa_tax() {
        let mut cpu = CPU::new();
        cpu.load_and_run(vec![0xaa,0x00]);
        assert_eq!(cpu.a_reg,cpu.x_reg);
    }

    #[test]
    fn test_0xe8_overflow_x_reg() {
        let mut cpu = CPU::new();
        cpu.load_and_run(vec![0xa9,0xff,0xaa,0xe8,0xe8,0x00]);
        assert_eq!(cpu.x_reg, 0x01);
    }

    #[test]
    fn test_zero_page_lda() {
        let mut cpu = CPU::new();
        cpu.mem_write(0x80, 0x14);
        cpu.load_and_run(vec![0xa5, 0x80, 0x00]);
        assert_eq!(cpu.a_reg, 0x14);
    }

    #[test]
    fn test_zero_page_x_lda() {
        let mut cpu = CPU::new();
        cpu.mem_write(0x82, 0x45);
        cpu.load_and_run(vec![0xa5,0x80,0xa2, 0x02,0xb5,0x80, 0x00]);
        assert_eq!(cpu.a_reg, 0x45);
    } 

    #[test]
    fn test_absolute_lda() {
        let mut cpu = CPU::new();
        cpu.mem_write(0x2000, 0x45);
        cpu.load_and_run(vec![0xAD, 0x00,0x20]);
        assert_eq!(cpu.a_reg, 0x45);
    }
    #[test]
    fn test_absolute_x_lda() {
        let mut cpu = CPU::new();
        cpu.mem_write(0x2001,0x45);
        cpu.load_and_run(vec![0xa2, 0x01,0xbd, 0x00,0x20]);
        assert_eq!(cpu.a_reg,0x45);
    }

    #[test]
    fn test_absolute_y_lda() {
        let mut cpu = CPU::new();
        cpu.mem_write(0x2001,0x45);
        cpu.load_and_run(vec![0xa0, 0x01,0xb9, 0x00,0x20]);
        assert_eq!(cpu.a_reg,0x45);
    }
    #[test]
    fn test_indexed_indirect_x_lda() {
        let mut cpu = CPU::new();
        cpu.mem_write_16(0x2000,0x0082);
        cpu.mem_write(0x2000,0x45);
        cpu.load_and_run(vec![0xa2, 0x02,0xa1, 0x80]);
        assert_eq!(cpu.a_reg,0x45);
    }

    #[test]
    fn test_indirect_indexed_y_lda() {
        let mut cpu = CPU::new();
        cpu.mem_write_16(0x2000, 0x0080);
        cpu.mem_write(0x2002,0x45);
        cpu.load_and_run(vec![0xa0,0x02,0xb1,0x80,0x00])
    }
    // #[test]
    // fn test_overflow_flag() {
    //     let mut cpu = CPU::new();
    //     cpu.a_reg = 0x7f;
    //     cpu.add_to_register_a(0x03);
    //     assert_eq!(cpu.a_reg, 0x82);
    //     assert!(cpu.status_reg.contains(CPUflags::OVERFLOW) == true);
    // }

    #[test]
    fn test_set_flag() {
        let mut cpu = CPU::new();
        cpu.set_flag(&CPUflags::CARRY);
        assert!(cpu.status_reg.contains(CPUflags::CARRY) == true);
    }
}

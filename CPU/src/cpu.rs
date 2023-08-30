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
            0xA9 | 0xA5 | 0xB5 | 0xad | 0xbd | 0xb9 | 0xa1 | 0xb1 => {
                self.lda(&opcode.mode);
            },
            0xE8 => {
                self.inx();
            },
            0xAA => {
                self.tax();
            },
            0xA2 | 0xa6 | 0xb6 | 0xae | 0xbe =>  {
                self.ldx(&AddressModes::immediate);

            },
            0xA0 | 0xa4 | 0xb4 | 0xac | 0xbc => {
                self.ldy(&AddressModes::immediate);
            },
            0x00 => {return;},
            _ => println!("Not this!!!"),
            
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
        //                                     {if a_reg and data have same sign and a_reg and temp have different sign, then overflow}
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
        data = data.wrapping_add(1);
        self.mem_write(operand_address, data);
    }

    //Shift and Rotate Instructions
    //Loading and storing from memeory
    //Flag clear
    //Flag set
    //Branching Instructions
    //Stack Instructions
    fn lda(&mut self , addressing_mode: &AddressModes) {
        let operand_address = self.get_operand_address(addressing_mode);
        self.a_reg = self.mem_read(operand_address);
        self.update_zero_and_negative_flags(self.a_reg);
        
    }


    fn tax(&mut self) {
        self.x_reg = self.a_reg;
        self.update_zero_and_negative_flags(self.x_reg);

    }
    fn ldx(&mut self, addressing_mode: &AddressModes) {
        let operand_address = self.get_operand_address(addressing_mode);
        self.x_reg = self.mem_read(operand_address);
        self.update_zero_and_negative_flags(self.x_reg);
    } 
   

    fn ldy(&mut self, addressing_mode: &AddressModes) {
        let operand_address = self.get_operand_address(addressing_mode);
        self.y_reg = self.mem_read(operand_address);
        self.update_zero_and_negative_flags(self.y_reg);
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
}

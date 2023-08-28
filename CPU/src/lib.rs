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

pub struct CPU {
    //Declaring general Purpose 8 bit register
    pub A_Reg: u8,
    pub X_reg: u8,
    pub Y_reg: u8,
    pub status_reg: u8, //8 bit status_register
    pub PC: u16, //16 bit Program Counter Register. Why 16 bit? Cause address line is 16 bit
    memory: [u8;0xFFFF] //Creating a memory space of 65535
}

impl CPU {
    pub fn new() -> Self {
        Self {
            A_Reg: 0,
            status_reg: 0,
            PC: 0,
            X_reg: 0,
            Y_reg: 0,
            memory: [0;0xFFFF]
        }
    }
//Instead of creating multiple memory read and write functions for 8 bit and 16 bit create a single function. Maybe use trait or maybe 
    fn mem_read(&self , address: u16) -> u8 {
        self.memory[address as usize]
    }

    fn mem_write(&mut self, address: u16, data: u8) {
        self.memory[address as usize] = data;
    }

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
        self.A_Reg = 0;
        self.X_reg =  0;
        self.Y_reg = 0;
        self.status_reg = 0;
        
        self.PC = self.mem_read_16(0xfffc);
    }

    pub fn run(&mut self) {
        
        loop {
        let opcode = self.mem_read(self.PC); 
        self.PC += 1;

        match opcode {
            //Group this under one mnemonic for each opcodes
            0xA9 => {
                self.lda(&AddressModes::immediate);
                self.PC += 1;
            },
            0xA5 => {
                self.lda(&AddressModes::zero_page);
                self.PC += 1;
            },
            0xB5 => {
                self.lda(&AddressModes::zero_page_x);
                self.PC += 1;
            },
            0xAD => {
                self.lda(&AddressModes::absolute);
                self.PC += 2;
            },
            0xBD => {
                self.lda(&AddressModes::absolute_x);
                self.PC += 2;
            }
            0xB9 => {
                self.lda(&AddressModes::absolute_y);
                self.PC += 2;
            },
            0xA1 => {
                self.lda(&AddressModes::Indexed_Indirect_x);
                self.PC += 1;
            },
            0xB1 => {
                self.lda(&AddressModes::Indirect_Indexed_y);
                self.PC += 1;
            },
            0xE8 => {
                self.inx();
            },
            0xAA => {
                self.tax();
            },
            0xA2 =>  {
                self.ldx(&AddressModes::immediate);
                self.PC += 1;

            },
            0xA0 => {
                self.ldy(&AddressModes::immediate);
                self.PC += 1;
            },
            0x00 => {return;},
            _ => println!("Not this!!!"),
            
        }
    }}



    fn update_zero_and_negative_flags(&mut self, current_result: u8) {
        //Setting the zero Flag
        if current_result == 0 {
            self.status_reg = self.status_reg | 0b0000_0010; // 0 flag MUST be set no matter what, and other flag unaffected, so OR operation is required
        }
        else {
            self.status_reg = self.status_reg & 0b1111_1101;
        }
        //setting the negative flag
        if current_result & 0b1000_0000 != 0 { // check the msb of the result
            self.status_reg = self.status_reg | 0b1000_0000; // MSB = 1, set NegativeFlag = 1
        }
        else {
            self.status_reg = self.status_reg & 0b0111_1111; 

        }
    }

    fn get_operand_address(&self, mode : &AddressModes) -> u16 {

        match mode {
            AddressModes::immediate => self.PC,
            AddressModes::zero_page => self.mem_read(self.PC) as u16,
            AddressModes::zero_page_x => {
                let pos = self.mem_read(self.PC);
                pos.wrapping_add(self.X_reg) as u16
            },
            AddressModes::zero_page_y => {
                let pos= self.mem_read(self.PC);
                pos.wrapping_add(self.Y_reg) as u16
            }
            AddressModes::absolute => self.mem_read_16(self.PC),
            AddressModes::absolute_x => {
                let base_address = self.mem_read_16(self.PC);
                base_address.wrapping_add(self.X_reg as u16)
            },
            AddressModes::absolute_y => {
                let base_address = self.mem_read_16(self.PC);
                base_address.wrapping_add(self.Y_reg as u16)
            },
            
            //This is a bit tricky.
            //                                                                        |self.PC is here|
            //What we are trying to do is LDA ($80,x); [opcodes] = [0xa9(lda opcode), 0x80(address), 0x00(next opcode)]
            //We ought to add 'x_register' to 80 say x is 2: so it becomes LDA ($82).... 
            // We need 2 byte address stored at 82 and 83 cause only one byte is stored at a memory cell:
            // lets say: |82| data: 00 (Lower Byte of address cause it is little endian)|
            //           |23| data: 20 (higher byte of the address)|
            // we simple do mem_read(82) (lowee byte) and mem_read(83) and combine the both using bit arthimetic to get the address of the operand


            AddressModes::Indexed_Indirect_x => {
                let base = self.mem_read(self.PC);
                let ptr = (base as u8).wrapping_add(self.X_reg) as u8;
                let lower_byte = self.mem_read(ptr as u16);
                let higher_byte = self.mem_read(ptr.wrapping_add(1) as u16);
                (higher_byte as u16) << 8 | (lower_byte as u16)
            },

            AddressModes::Indirect_Indexed_y => {
                let base = self.mem_read(self.PC);
                let lowerByte_address = self.mem_read(base as u16);
                let higherByte_address = self.mem_read((base.wrapping_add(1)) as u16);
                let address = (higherByte_address as u16) << 8 | (lowerByte_address as u16);
                address.wrapping_add(self.Y_reg as u16)
            },
            AddressModes::NonaddressingMode => {
                panic!("Mode {:?} is not supported",mode)
            }

        }

    }

    //Opcodes
    fn lda(&mut self , addressingMode: &AddressModes) {
        let operand_address = self.get_operand_address(addressingMode);
        self.A_Reg = self.mem_read(operand_address);
        self.update_zero_and_negative_flags(self.A_Reg);
        
    }

    fn inx(&mut self) {
        self.X_reg = self.X_reg.wrapping_add(1);
        self.update_zero_and_negative_flags(self.X_reg);
    }

    fn tax(&mut self) {
        self.X_reg = self.A_Reg;
        self.update_zero_and_negative_flags(self.X_reg);

    }
    fn ldx(&mut self, addressingMode: &AddressModes) {
        let operand_address = self.get_operand_address(addressingMode);
        self.X_reg = self.mem_read(operand_address);
        self.update_zero_and_negative_flags(self.X_reg);
    } 

    fn ldy(&mut self, addressingMode: &AddressModes) {
        let operand_address = self.get_operand_address(addressingMode);
        self.Y_reg = self.mem_read(operand_address);
        self.update_zero_and_negative_flags(self.Y_reg);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_LDA_immediate_load() {
        let mut cpu = CPU::new();
        let instructions = vec![0xa9,0x05, 0x00];
        cpu.load_and_run(instructions);

        assert_eq!(cpu.A_Reg, 0x05);
        assert!(cpu.status_reg & 0b1000_0000 == 0);

    }

    #[test]
    fn test_0xa9_lda_zero_flag() {
        let mut cpu = CPU::new();
        cpu.load_and_run(vec![0xa9, 0x00, 0x00]);
        assert!(cpu.status_reg & 0b0000_0010 == 0b10);
    }
    #[test]
    fn test_0xa0_lda_negative_flag() {
        let mut cpu = CPU::new();
        cpu.load_and_run(vec![0xa9,0xff,0x00]);
        assert!(cpu.status_reg & 0b1000_0000 == 0b1000_0000);
    }
    #[test]
    fn test_Ops_working_together() {
        let mut cpu = CPU::new();
        cpu.load_and_run(vec![0xa9,0x05,0xaa,0xe8,0x00]);
        assert_eq!(cpu.X_reg, 0x06);
    }

    #[test]
    fn test_0xaa_TAX() {
        let mut cpu = CPU::new();
        cpu.load_and_run(vec![0xaa,0x00]);
        assert_eq!(cpu.A_Reg,cpu.X_reg);
    }

    #[test]
    fn test_0xe8_overflow_x_reg() {
        let mut cpu = CPU::new();
        cpu.load_and_run(vec![0xa9,0xff,0xaa,0xe8,0xe8,0x00]);
        assert_eq!(cpu.X_reg, 0x01);
    }

    #[test]
    fn test_Zero_page_LDA() {
        let mut cpu = CPU::new();
        cpu.mem_write(0x80, 0x14);
        cpu.load_and_run(vec![0xa5, 0x80, 0x00]);
        assert_eq!(cpu.A_Reg, 0x14);
    }

    #[test]
    fn test_Zero_page_X_LDA() {
        let mut cpu = CPU::new();
        cpu.mem_write(0x82, 0x45);
        cpu.load_and_run(vec![0xa5,0x80,0xa2, 0x02,0xb5,0x80, 0x00]);
        assert_eq!(cpu.A_Reg, 0x45);
    } 

    #[test]
    fn test_absolute_LDA() {
        let mut cpu = CPU::new();
        cpu.mem_write(0x2000, 0x45);
        cpu.load_and_run(vec![0xAD, 0x00,0x20]);
        assert_eq!(cpu.A_Reg, 0x45);
    }
    #[test]
    fn test_absolute_X_LDA() {
        let mut cpu = CPU::new();
        cpu.mem_write(0x2001,0x45);
        cpu.load_and_run(vec![0xa2, 0x01,0xbd, 0x00,0x20]);
        assert_eq!(cpu.A_Reg,0x45);
    }

    #[test]
    fn test_absolute_Y_LDA() {
        let mut cpu = CPU::new();
        cpu.mem_write(0x2001,0x45);
        cpu.load_and_run(vec![0xa0, 0x01,0xb9, 0x00,0x20]);
        assert_eq!(cpu.A_Reg,0x45);
    }
    #[test]
    fn test_Indexed_Indirect_X_LDA() {
        let mut cpu = CPU::new();
        cpu.mem_write_16(0x2000,0x0082);
        cpu.mem_write(0x2000,0x45);
        cpu.load_and_run(vec![0xa2, 0x02,0xa1, 0x80]);
        assert_eq!(cpu.A_Reg,0x45);
    }

    #[test]
    fn test_Indirect_Indexed_Y_LDA() {
        let mut cpu = CPU::new();
        cpu.mem_write_16(0x2000, 0x0080);
        cpu.mem_write(0x2002,0x45);
        cpu.load_and_run(vec![0xa0,0x02,0xb1,0x80,0x00])
    }
}

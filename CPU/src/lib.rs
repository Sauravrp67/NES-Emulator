pub enum AddressModes {
    immediate,
    zero_page,
    zeropage_x,
    zero_page_y,
    absolute,
    absolute_x,
    absolute_y,
    Indirect_x,
    Indirect_y

}



pub struct CPU {
    //Declaring general Purpose 8 bit register
    pub A_Reg: u8,
    pub X_reg: u8,
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
            memory: [0;0xFFFF]
        }
    }

    fn mem_read(&mut self , address: u16) -> u8 {
        self.memory[address as usize]
    }

    fn mem_write(&mut self, address: u16, data: u8) {
        self.memory[address as usize] = data;
    }

    fn mem_read_16(&mut self, address: u16) -> u16 {
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
        self.status_reg = 0;
        
        self.PC = self.mem_read_16(0xfffc)
    }

    pub fn run(&mut self) {
        
        loop {
        let opcode = self.mem_read(self.PC); 

        self.PC += 1;
        }
    }



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

    //LDA operation
    fn lda(&mut self , value: u8) {
        self.A_Reg = value;
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

    pub fn interpret(&mut self, program: Vec<u8>) {
        self.PC = 0;

        loop {
            //This is fetching cycle
            let opcode = program[self.PC as usize];
            self.PC += 1;
            //This is decoding
            match opcode {
                0xa9 => {
                    let param = program[self.PC as usize];

                    self.PC += 1;

                    self.lda(param);
                },
                0xaa => {
                    self.tax();
                }
                0xe8 => self.inx(),
                0x00 => return,
                 _ => todo!(),


            }
        }
    } 
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_LDA_immediate_load() {
        let mut cpu = CPU::new();
        let instructions = vec![0xa9,0x05, 0x00];
        cpu.interpret(instructions);

        assert_eq!(cpu.A_Reg, 0x05);
        assert!(cpu.status_reg & 0b1000_0000 == 0);

    }

    #[test]
    fn test_0xa9_lda_zero_flag() {
        let mut cpu = CPU::new();
        cpu.interpret(vec![0xa9, 0x00, 0x00]);
        assert!(cpu.status_reg & 0b0000_0010 == 0b10);
    }
    #[test]
    fn test_0xa0_lda_negative_flag() {
        let mut cpu = CPU::new();
        cpu.interpret(vec![0xa9,0xff,0x00]);
        assert!(cpu.status_reg & 0b1000_0000 == 0b1000_0000);
    }
    #[test]
    fn test_Ops_working_together() {
        let mut cpu = CPU::new();
        cpu.interpret(vec![0xa9,0x05,0xaa,0xe8,0x00]);
        assert_eq!(cpu.X_reg, 0x06);
    }

    #[test]
    fn test_0xaa_TAX() {
        let mut cpu = CPU::new();
        cpu.interpret(vec![0xaa,0x00]);
        assert_eq!(cpu.A_Reg,cpu.X_reg);
    }

    #[test]
    fn test_0xe8_overflow_x_reg() {
        let mut cpu = CPU::new();
        cpu.interpret(vec![0xa9,0xff,0xaa,0xe8,0xe8,0x00]);
        assert_eq!(cpu.X_reg, 0x01);
    }
}

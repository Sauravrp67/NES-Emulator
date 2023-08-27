pub struct CPU {
    //Declaring general Purpose 8 bit register
    pub A_Reg: u8,
    pub X_reg: u8,
    pub status_reg: u8, //8 bit status_register
    pub PC: u16, //16 bit Program Counter Register. Why 16 bit? Cause address line is 16 bit
}

impl CPU {
    pub fn new() -> Self {
        Self {
            A_Reg: 0,
            status_reg: 0,
            PC: 0,
            X_reg: 0,
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
        self.X_reg = self.A_Reg.wrapping_add(1);
    }

    fn tax(&mut self) {
        self.X_reg = self.A_Reg;

    }

    pub fn interpret(&mut self, program: Vec<u8>) {
        self.PC = 0;

        loop {
            let opcode = program[self.PC as usize];
            self.PC += 1;

            match opcode {
                0xa9 => {
                    let param = program[self.PC as usize];

                    self.PC += 1;

                    self.lda(param);
                },
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
}

pub mod opcode {

use std::collections::HashMap;
use crate::cpu::cpu::AddressModes;

pub struct OpCode {
    pub code: u8,
    pub mnemonic: &'static str,
    pub len: u8,
    pub cycles: u8,
    pub mode: AddressModes
}

impl OpCode {
    fn new(code: u8,mnemonic : &'static str,len : u8, cycles: u8, mode: AddressModes) -> Self {
        OpCode {
            code,
            mnemonic,
            len,
            cycles,
            mode
        }
    }
}

lazy_static! {


    //static data structures that are computed at runtime, rather than compile time, and are initialized lazily.
    //This is particularly useful for cases where you want to initialize complex data structures only when they are first used. 
    static ref OP_CODES:Vec<OpCode> =  
    vec![
    //Implied Addressing Mode OpCodes
    OpCode::new(0x00,"BRK",1,7, AddressModes::NonaddressingMode),
    OpCode::new(0x18,"CLC",1,2, AddressModes::NonaddressingMode),
    OpCode::new(0xD8,"CLD",1,2, AddressModes::NonaddressingMode),
    OpCode::new(0x58,"CLI",1,2, AddressModes::NonaddressingMode),
    OpCode::new(0xb8,"CLV",1,2,AddressModes::NonaddressingMode),
    OpCode::new(0xca,"DEX",1,2,AddressModes::NonaddressingMode),
    OpCode::new(0x88,"DEY",1,2,AddressModes::NonaddressingMode),
    OpCode::new(0xe8,"INX",1,2,AddressModes::NonaddressingMode),
    OpCode::new(0xc8,"INY",1,2,AddressModes::NonaddressingMode),
    OpCode::new(0xea,"NOP",1,2,AddressModes::NonaddressingMode),
    OpCode::new(0x48,"PHA",1,2,AddressModes::NonaddressingMode),
    OpCode::new(0x08,"PHP",1,2,AddressModes::NonaddressingMode),
    OpCode::new(0x68,"PLA",1,4,AddressModes::NonaddressingMode),
    OpCode::new(0x28,"PLP",1,4,AddressModes::NonaddressingMode),
    OpCode::new(0x40,"RTI",1,6,AddressModes::NonaddressingMode),
    OpCode::new(0x60,"RTS",1,6,AddressModes::NonaddressingMode),
    OpCode::new(0x38,"SEC",1,2,AddressModes::NonaddressingMode),
    OpCode::new(0xf8,"SED",1,2,AddressModes::NonaddressingMode),
    OpCode::new(0x78,"SEI",1,2,AddressModes::NonaddressingMode),
    OpCode::new(0xaa,"TAX",1,2,AddressModes::NonaddressingMode),
    OpCode::new(0xa8,"TAY",1,2,AddressModes::NonaddressingMode),
    OpCode::new(0xba,"TSX",1,2,AddressModes::NonaddressingMode),
    OpCode::new(0x8a,"INY",1,2,AddressModes::NonaddressingMode),
    OpCode::new(0x9a,"TXS",1,2,AddressModes::NonaddressingMode),
    OpCode::new(0x98,"TYA",1,2,AddressModes::NonaddressingMode),
    //
    //LDA    
    OpCode::new(0xa9,"LDA", 2, 2, AddressModes::immediate),
    OpCode::new(0xa5,"LDA", 2, 2, AddressModes::zero_page),
    OpCode::new(0xb5,"LDA", 2, 2, AddressModes::zero_page_x),
    OpCode::new(0xad,"LDA", 3, 2, AddressModes::absolute),
    OpCode::new(0xbd,"LDA", 3, 2, AddressModes::absolute_x),
    OpCode::new(0xb9,"LDA", 3, 2, AddressModes::absolute_y),
    OpCode::new(0xa1,"LDA", 2, 2, AddressModes::Indexed_Indirect_x),
    OpCode::new(0xb1,"LDA", 2, 2, AddressModes::Indirect_Indexed_y),

    //LDX
    OpCode::new(0xa2,"LDX", 2, 2, AddressModes::immediate),
    OpCode::new(0xa6,"LDX", 2, 3, AddressModes::zero_page),
    OpCode::new(0xb6,"LDX", 2, 4, AddressModes::zero_page_y),
    OpCode::new(0xae,"LDX", 3, 4, AddressModes::absolute),
    OpCode::new(0xbe,"LDX", 3, 4, AddressModes::absolute_y),

    //LDY
    OpCode::new(0xa0,"LDY",2,2,AddressModes::immediate),
    OpCode::new(0xa4,"LDY",2,3,AddressModes::zero_page),
    OpCode::new(0xb4,"LDX",2,4,AddressModes::zero_page_x),
    OpCode::new(0xac,"LDX",3,4,AddressModes::absolute),
    OpCode::new(0xbc,"LDX",3,4,AddressModes::absolute_x)
    ];


    pub static ref OPCODES_MAP: HashMap<u8,&'static OpCode>= {
        let mut map = HashMap::new();
        for cpuop in &*OP_CODES/*Deferences the reference to Vec<OpCode> and then takes reference to borrowed value*/  {
            map.insert(cpuop.code,cpuop);
        }
        map
    };

}
}


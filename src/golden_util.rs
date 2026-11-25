//! Test-only helpers for reading `c-golden/golden_misc.txt`.
//! Line format:  `<kind> <name> : <harness args> : <KEY=hex bytes of the body under test> : <out1>|<out2>|...`
//! where the outputs are what the REAL C function printed (decoded fields, RESULT ok/error, SENT <frame>, HOSTBAUD n).
#![cfg(test)]

pub const GOLDEN: &str = include_str!("../c-golden/golden_misc.txt");

pub struct Case {
    pub name: String,
    pub args: Vec<String>,
    pub body: Vec<u8>,
    pub out: Vec<String>,
}

pub fn hex(s: &str) -> Vec<u8> {
    s.split_whitespace().map(|h| u8::from_str_radix(h, 16).unwrap()).collect()
}

pub fn cases(kind: &str) -> Vec<Case> {
    GOLDEN
        .lines()
        .filter(|l| l.split_whitespace().next() == Some(kind))
        .map(|l| {
            let p: Vec<&str> = l.splitn(4, " : ").collect();
            let name = p[0].split_whitespace().nth(1).unwrap().to_string();
            let args = p[1].split_whitespace().map(String::from).collect();
            let conf = p[2].split_once('=').unwrap().1;
            Case { name, args, body: if conf == "-" { vec![] } else { hex(conf) }, out: p[3].split('|').map(String::from).collect() }
        })
        .collect()
}

impl Case {
    pub fn ok(&self) -> bool {
        self.out.iter().any(|o| o == "RESULT ok")
    }
    /// The single output line starting with `prefix`, without it.
    pub fn find(&self, prefix: &str) -> Option<&str> {
        self.out.iter().find_map(|o| o.strip_prefix(prefix))
    }
    pub fn all(&self, prefix: &str) -> Vec<&str> {
        self.out.iter().filter_map(|o| o.strip_prefix(prefix)).collect()
    }
}

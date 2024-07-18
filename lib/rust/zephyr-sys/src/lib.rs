#![no_std]

#![allow(non_upper_case_globals)]
#![allow(non_camel_case_types)]
#![allow(non_snake_case)]
#![allow(improper_ctypes)]

use core::include;
use core::concat;
use core::env;

include!(concat!(env!("OUT_DIR"), "/bindings.rs"));
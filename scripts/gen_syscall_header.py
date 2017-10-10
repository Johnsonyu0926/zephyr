#!/usr/bin/env python3
#
# Copyright (c) 2017 Intel Corporation
#
# SPDX-License-Identifier: Apache-2.0

import sys

def gen_macro(ret, argc):
    sys.stdout.write("K_SYSCALL_DECLARE%d%s(id, name" % (argc,
        ("" if ret else "_VOID")))
    if (ret):
        sys.stdout.write(", ret")
    for i in range(argc):
        sys.stdout.write(", t%d, p%d" % (i, i))
    sys.stdout.write(")")

def gen_fn(ret, argc, name, extern=False):
    sys.stdout.write("\t%s %s %s(" %
            (("extern" if extern else "static inline"),
             ("ret" if ret else "void"), name))
    if argc == 0:
        sys.stdout.write("void");
    else:
        for i in range(argc):
            sys.stdout.write("t%d p%d" % (i, i))
            if i != (argc - 1):
                sys.stdout.write(", ")
    sys.stdout.write(")")

def gen_make_syscall(ret, argc):
    if (ret):
        sys.stdout.write("return (ret)")
    if (argc <= 6):
        sys.stdout.write("_arch")
    sys.stdout.write("_syscall_invoke%d(" % (argc))
    for i in range(argc):
        sys.stdout.write("(u32_t)p%d, " % (i))
    sys.stdout.write("id); \\\n")

def gen_call_impl(ret, argc):
    if (ret):
        sys.stdout.write("return ")
    sys.stdout.write("_impl_##name(")
    for i in range(argc):
        sys.stdout.write("p%d" % (i))
        if i != (argc - 1):
            sys.stdout.write(", ")
    sys.stdout.write("); \\\n")

def newline():
    sys.stdout.write(" \\\n")

def gen_defines_inner(ret, argc, kernel_only=False, user_only=False):
    sys.stdout.write("#define ")
    gen_macro(ret, argc)
    newline()

    if not user_only:
        gen_fn(ret, argc, "_impl_##name", extern=True)
        sys.stdout.write(";")
        newline()

    gen_fn(ret, argc, "name");
    newline()
    sys.stdout.write("\t{")
    newline()

    if kernel_only:
        sys.stdout.write("\t\t")
        gen_call_impl(ret, argc)
    elif user_only:
        sys.stdout.write("\t\t")
        gen_make_syscall(ret, argc)
    else:
        sys.stdout.write("\t\tif (_is_user_context()) {")
        newline()

        sys.stdout.write("\t\t\t")
        gen_make_syscall(ret, argc)

        sys.stdout.write("\t\t} else {")
        newline()

        sys.stdout.write("\t\t\t")
        gen_call_impl(ret, argc)
        sys.stdout.write("\t\t}")
        newline()

    sys.stdout.write("\t}\n\n")

def gen_defines(argc, kernel_only=False, user_only=False):
    gen_defines_inner(False, argc, kernel_only, user_only)
    gen_defines_inner(True, argc, kernel_only, user_only)


sys.stdout.write("/* Auto-generated by gen_syscall_header.py, do not edit! */\n\n")
sys.stdout.write("#ifndef GEN_SYSCALL_H\n#define GEN_SYSCALL_H\n\n")

for i in range(11):
    sys.stdout.write("#if !defined(CONFIG_USERSPACE) || defined(__ZEPHYR_SUPERVISOR__)\n")
    gen_defines(i, kernel_only=True)
    sys.stdout.write("#elif defined(__ZEPHYR_USER__)\n")
    gen_defines(i, user_only=True)
    sys.stdout.write("#else /* mixed kernel/user macros */\n")
    gen_defines(i)
    sys.stdout.write("#endif /* mixed kernel/user macros */\n\n")

sys.stdout.write("#endif /* GEN_SYSCALL_H */\n")






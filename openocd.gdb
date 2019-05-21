target extended-remote :3333

# set print asm-demangle on

# detect unhandled exceptions, hard faults and panics
break HardFault
break rust_begin_unwind

# *try* to stop at the user entry point (it might be gone due to inlining)
<<<<<<< HEAD
break main
=======
#break main
>>>>>>> d3ef1a1e552ab15e1c183e1cf6ea77fd759784a4

#monitor arm semihosting enable
#monitor semihosting ioclient 3

load

# start the process but immediately halt the processor
stepi

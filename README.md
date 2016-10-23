# LoRa-HopeRF96-with-Nucleo-board
LaRa module from HopeRF RFM96 transmitter and receiver


For setup LoRa registers -receiver & transmitter connect to any terminal programm
with boudrate 115200. Set COM port number with correspondent STlink Virtual COM port on NUCLEO board.

Send command in format


command  S(0xaa,0xdd)  send to registers

aa - hex number of the address of LoRa registers
dd - hex number for write to correspondent LoRa registers
after send this command device send to terminal "OK"
address & data for command set with correspondent addr & data from LoRa module datasheet


command  F(abcdef)  set frequency

abcdef – frequency in decimal format ( example  433175  -  is 433,175 MHz )


command  P(p)  set power    1- min 1,  7 - max

p - decimal digit from 1 to 7


command  B(b)   set signal bandwidth 1 - 7.8 kHz, 9 - 500 kHz

b - decimal digit from 1 to 9


command  D(d)   set Spreading Factor   1 - 64 chips / symbol, 7 - 4096 chips / symbol

d - decimal digit from 1 to 7


command  T(tt)   set  time for transmitter

tt - decimal digits from 01 to 99


command  L(lll)   set  length for transmitter

lll - decimal digits from 01 to 127


command  R(0xrr)   read rigister of LoRa device from 0x00 to 0x3F (hex mode)

rr - serial number of the register ( from datasheet ) in hex mode ( from 00 to 3F )

Length of all command 12 byte  !!!
If number of byte of command < 12 byte - add space code (0x20 hex)



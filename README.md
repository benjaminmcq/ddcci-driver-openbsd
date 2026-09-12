this driver implements backlight control for external monitors on OpenBSD.

it is explicitly not in the OpenBSD tree yet in any way.

the driver is actually very odd. it's not a character device because it doesn't take any input and function like something similar to /dev/ddcci.
it's a pseudo device, yet it does i2c commands.
it's not an i2c device, though i would propose it to the directory sys/dev/i2c on the tree.

the goal is to be able to control it from wsconsctl similar to how abl(4) is architected.

if you'd like to compile it for yourself, i'd suggest looking at documentation, which i won't link here.

thank you

NOTE: i don't test commits before committing on this branch.

dtb-$(CONFIG_DTB_BAIKAL_BFK)	+= baikal_bfk.dtb
dtb-$(CONFIG_DTB_BAIKAL_BFK3)	+= baikal_bfk3.dtb baikal_bfk2.dtb
dtb-$(CONFIG_DTB_BAIKAL_BFKX)	+= baikal_bfkx.dtb
dtb-$(CONFIG_DTB_BAIKAL_FPGA)	+= baikal_fpga.dtb
dtb-$(CONFIG_DTB_BAIKAL_QEMU)	+= baikal_qemu.dtb


dtb-$(CONFIG_DT_NONE)	+= \
						baikal_bfk.dtb	\
						baikal_bfk3.dtb	\
						baikal_bfkx.dtb	\
						baikal_fpga.dtb	\
						baikal_qemu.dtb

obj-y				+= $(patsubst %.dtb, %.dtb.o, $(dtb-y))

# Force kbuild to make empty built-in.o if necessary
obj-				+= dummy.o

always				:= $(dtb-y)
clean-files			:= *.dtb *.dtb.S

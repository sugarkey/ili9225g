obj-m += st7775r.o

KERN_DIR = /home/t15/Ingenic-SDK-3.2.2-20160229/opensource/kernel_recorder 

all:
	make -C $(KERN_DIR) M=`pwd` modules
clean:
	make -C $(KERN_DIR) M=`pwd` modules clean
	rm -rf modules.order


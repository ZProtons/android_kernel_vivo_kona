import os, sys, struct
import argparse

'''
struct blog_desc {
	u32 start;          // BYTE
	u32 size;           // BYTE
	u32 ring_offset;    // BYTE, partial ring offset
	u32 wptr;           // BYTE
};

struct blog_info {
	u32 count;
	u32 boot_complete;
	struct blog_desc stage[BLOG_REC_MAX];
};

struct blog_header {
	u32 magic;
	u32 version;
	u32 crc;
	u32 count;
	u32 flush_mode;
	u32 control;
	struct blog_info log[BLOG_NUM_MAX];
};
'''

class blog_parser:
	def __init__(self, input_file, output_dir=None, offset=0):
		self.input_file = input_file
		if output_dir is None:
			self.output_dir = os.path.dirname(input_file)
		else:
			self.output_dir = output_dir
		self.offset = offset

	def get_data(self):
		f = open(self.input_file, 'rb+')
		f.seek(self.offset)
		data = f.read()
		f.close()
		return data

	def parse(self):
		data = self.get_data()
		sizeof_u32 = 4
		BLOG_REC_MAX = 4
		BLOG_NUM_MAX = 10
		blog_desc_size = sizeof_u32*4
		blog_info_size = BLOG_REC_MAX * blog_desc_size + sizeof_u32*2
		blog_info_u32_size = blog_info_size - BLOG_REC_MAX * blog_desc_size
		blog_header_size = BLOG_NUM_MAX * blog_info_size + sizeof_u32*6
		blog_header_u32_size = blog_header_size - BLOG_NUM_MAX * blog_info_size
		blog_header_crc_offset = sizeof_u32*3

		blog_header = struct.unpack("IIIIII", data[0:blog_header_u32_size]);
		print 'blog_header:'
		print '\tmagic:\t\t\t0x%x' % blog_header[0]
		print '\tversion:\t\t0x%x' % blog_header[1]
		unpack_format = "%dI" % ((blog_header_size - blog_header_crc_offset) / sizeof_u32)
		crc_data = struct.unpack(unpack_format, data[blog_header_crc_offset : blog_header_size])
		print '\tcrc:\t\t\t0x%08x (actual: 0x%08x)' % (blog_header[2], sum(crc_data) & 0xFFFFFFFF)
		print '\tcount:\t\t\t0x%x' % blog_header[3]
		current_count = blog_header[3]
		print '\tflush_mode:\t\t0x%x' % blog_header[4]
		print '\tcontrol:\t\t0x%x' % blog_header[5]

		log_title = ["sbl log:", "UEFI log:", "Kernel step:", "Android step:"]
		for i in range(0, BLOG_NUM_MAX):
			blog_info = struct.unpack("II", data[blog_header_u32_size + blog_info_size * i : \
				blog_header_u32_size + blog_info_u32_size + blog_info_size * i]);
			print 'blog_info[%d]:' % i
			print '\tcount:\t\t\t0x%x' % blog_info[0]
			print '\tboot_complete\t\t0x%x' % blog_info[1]
                        if blog_info[0] == 0xFFFFFFFF:
                            continue
			output_file = os.path.join(self.output_dir, os.path.join('logdump_%d.log' % (current_count - blog_info[0])))
			f = open(output_file, 'w')
			for j in range(0, BLOG_REC_MAX):
				blog_desc = struct.unpack("IIII", data[blog_header_u32_size + blog_info_u32_size + \
					blog_info_size * i + blog_desc_size *j : \
					blog_header_u32_size + blog_info_u32_size + blog_desc_size + blog_info_size * i + blog_desc_size * j]);
				print '\tblog_desc[%d]:' % j
				print '\t\tstart:\t\t0x%x' % blog_desc[0]
				print '\t\tsize:\t\t0x%x' % blog_desc[1]
				print '\t\tring_offset:\t0x%x' % blog_desc[2]
				print '\t\twptr:\t\t0x%x' % blog_desc[3]
				f.write(log_title[j] + "\n" + data[blog_desc[0] : blog_desc[0] + blog_desc[1]].split('\0')[0] + "\n\n")
			f.close()


def __blog_args_setup__():
	usage = "blog_parser.py"
	parser = argparse.ArgumentParser(usage)
	parser.add_argument('logdump', help='logdump')
	parser.add_argument('--output', help='output path', default=None, required=False)
	parser.add_argument('--offset', help='blog partition offset', default='0x1000', required=False)
	return parser.parse_args(sys.argv[1:])


if __name__ == '__main__':
	args = __blog_args_setup__()
	p = blog_parser(args.logdump, output_dir=args.output, offset=int(args.offset, 16))
	p.parse()


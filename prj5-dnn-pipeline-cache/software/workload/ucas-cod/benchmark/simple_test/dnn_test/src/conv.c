#include "printf.h"
#include "trap.h"
#include "mul.h"
#include "div.h"
#include "perf_cnt.h"

#define FRAC_BIT 10 // 10-bit fractional part

#define RD_ADDR 135106448 // input image address
#define RD_SIZE_D0 1      // no use
#define RD_SIZE_D1 1      // input image channel size
#define RD_SIZE_D2 28     // input image height
#define RD_SIZE_D3 28     // input image width

#define WEIGHT_ADDR 134217728 // convolution kernel address
#define WEIGHT_SIZE_D0 20     // convolution kernel groups number (output image channel size)
#define WEIGHT_SIZE_D1 1      // convolution kernel number (input image channel size)
#define WEIGHT_SIZE_D2 5      // convolution kernel height
#define WEIGHT_SIZE_D3 5      // convolution kernel width

#define WR_ADDR 135108240 // output image address
#define WR_SIZE_D0 1      // no use
#define WR_SIZE_D1 20     // output image channel size
#define WR_SIZE_D2 12     // output image height
#define WR_SIZE_D3 12     // output image width

#define KERN_ATTR_CONV_PAD 0       // convolution padding
#define KERN_ATTR_CONV_STRIDE 1    // convolution stride
#define KERN_ATTR_POOL_PAD 0       // pooling padding
#define KERN_ATTR_POOL_KERN_SIZE 2 // pooling kernel size
#define KERN_ATTR_POOL_STRIDE 2    // pooling stride

//MMIO register address of DNN accelerator
#define GPIO_START_ADDR    0x60030000 // hardware accelerator start address
#define GPIO_DONE_ADDR     0x60030008 // hardware accelerator done address

// minimum numbers
#define MIN_SHORT 0x8000 // minimum number of signed short

struct size_vec4 // 4-dimension array size
{
	unsigned d0;
	unsigned d1;
	unsigned d2;
	unsigned d3;
};

struct mem_addr // memory address
{
	unsigned rd_addr;
	unsigned weight_addr;
	unsigned wr_addr;
};

static inline
int mul(short a, short b) // 16-bit multiplication 
{
#ifndef USE_MUL
	int ans = mul_ll(a, b); // this function is changed to quicken calculation
#else
	int ans = a * b;
#endif
	return ans;
}

struct mem_addr addr = {RD_ADDR, WEIGHT_ADDR, WR_ADDR}; // memory address
struct size_vec4 rd_size = {RD_SIZE_D0, RD_SIZE_D1, RD_SIZE_D2, RD_SIZE_D3}; // input image size [1,1,28,28]
struct size_vec4 wr_size = {WR_SIZE_D0, WR_SIZE_D1, WR_SIZE_D2, WR_SIZE_D3}; // output image size [1,20,12,12]
struct size_vec4 weight_size = {WEIGHT_SIZE_D0, WEIGHT_SIZE_D1, WEIGHT_SIZE_D2, WEIGHT_SIZE_D3}; // convolution kernel size [20,1,5,5]

struct size_vec4 conv_size; // convolution output image size [1,20,24,24]

extern char _binary_data_result_bin_start[]; // golden output image
extern char _binary_data_result_bin_size[];  // golden output image size

void convolution()  // convolution
{
	short *in = (short *)addr.rd_addr;         // address of array: input image
	short *weight = (short *)addr.weight_addr; // address of array: convolution kernel
	short *out = (short *)addr.wr_addr;        // address of array: output image

	unsigned output_offset = 0; // output image offset
	// unsigned output_offset = mul(mul(WR_SIZE_D0, WR_SIZE_D1), mul(WR_SIZE_D2, WR_SIZE_D3)); // output image offset
	unsigned input_offset = 0;  // input image offset

	unsigned input_fm_w = rd_size.d3; // input image width
	unsigned input_fm_h = rd_size.d2; // input image height

	unsigned pad = KERN_ATTR_CONV_PAD; // padding length
	unsigned pad_len = pad << 1;       // padding length * 2

	unsigned conv_out_w = rd_size.d3 - weight_size.d3 + pad_len; // initialize output image width
	unsigned conv_out_h = rd_size.d2 - weight_size.d2 + pad_len; // initialize output image height

	unsigned stride = KERN_ATTR_CONV_STRIDE; // stride

	conv_out_w = div(conv_out_w, stride);
	conv_out_h = div(conv_out_h, stride);

	conv_out_w++; // output image width  = (ix - kx + pad * 2) / stride + 1
	conv_out_h++; // output image height = (iy - ky + pad * 2) / stride + 1

	conv_size.d0 = wr_size.d0; // no use / input image channel size
	conv_size.d1 = wr_size.d1; // output image channel size
	conv_size.d2 = conv_out_h; // output image height
	conv_size.d3 = conv_out_w; // output image width

	//TODO: Please add your implementation here

	unsigned oc; // output channel
	unsigned oy; // output height
	unsigned ox; // output width
	unsigned ic; // input channel
	int iy_w; // input height
	int iy; // input height
	int ix; // input width
	unsigned ky; // kernel height
	unsigned kx; // kernel width
	int sum;  // sum of convolution for one pixel

	/*
		For each output channel, chose each input channel.
		At each pixel, first set bias into output when ic==0.
		Then, calculate the corresponding input pixel, and for each conv_kernel pixel, multiply and add them together.
		Put the sum into the corresponding output pixel.
		Attention: kernel has this format: kernel[oc][ic][ky*kx+1]
			  kernel has this size: kernel[d0][d1][d2*d3+1]
	*/

	unsigned input_size_hw = mul(input_fm_h, input_fm_w); // input image height * input image width
	unsigned weight_size_hw = mul(weight_size.d2, weight_size.d3) + 1; // convolution kernel height * convolution kernel width + 1
	unsigned weight_size_chw = mul(weight_size_hw, weight_size.d1); // input image channel * 26
	unsigned conv_size_hw = mul(conv_size.d2, conv_size.d3); // output image height * output image width

	/*
	for (oc = 0; oc < 20; oc++) // output channel size
	{
		for (oy = 0; oy < 24; oy++) // output image height
		{
			for (ox = 0; ox < 24; ox++) // output image width
			{
				// -----out[oc][oy][ox] = weight[oc][0][0]-----
				// *(out + output_offset + mul(mul(oc, conv_size.d2), conv_size.d3) + mul(oy, conv_size.d3) + ox) = *weight; // initialize output image
				// *(out + output_offset + mul((short)(mul(oc, conv_size.d2)) + oy, conv_size.d3) + ox) = *(weight + mul((short)(mul((short)(mul(weight_size.d2, weight_size.d3)) + 1, weight_size.d1)), oc)); // initialize output image
				sum = 0;
				for (ic = 0; ic < 1; ic++) // input channel size
				{
					for (ky = 0; ky < 5; ky++) // kernel height
					{
						for (kx = 0; kx < 5; kx++) // kernel width
						{
							iy = ky + oy; // calculate input image height
							ix = kx + ox; // calculate input image width
							if (iy < 0 || iy >= 28 || ix < 0 || ix >= 28) // ignore padding
								continue;
							// -----out[oc][oy][ox] += in[ic][iy][ix] * weight[oc][ic][ky*kx+1]]-----
							sum += mul(in[mul(ic, 784) + mul(iy, 28) + ix],
								   weight[mul(oc, 26) + mul(ky, 5) + kx + 1]);
							// sum += mul(*(in + input_offset + mul(mul(ic, input_fm_h) + iy, input_fm_w) + ix),
							// 		   *(weight + mul(mul(oc, weight_size.d1) + ic, mul(weight_size.d2, weight_size.d3) + 1) + mul(ky, weight_size.d3) + kx + 1));
						}
					}
				}
				out[mul(oc, 576) + mul(oy, 24) + ox] =
					(short)(sum >> 10) + weight[mul(26, oc)];
			}
		}
	}
	*/

	int ic_ihw;
	int oc_chw;
	int oc_wchw;
	int ky_ww;
	int oy_cw;

	oc_chw = 0;
	oc_wchw = 0;
	for (oc = 0; oc < conv_size.d1; oc++) // output channel size
	{
		ic_ihw = 0;
		for (ic = 0; ic < rd_size.d1; ic++) // input channel size
		{
			oy_cw = 0;
			for (oy = 0; oy < conv_size.d2; oy++) // output image height
			{
				for (ox = 0; ox < conv_size.d3; ox++) // output image width
				{
					// -----out[oc][oy][ox] = weight[oc][0][0]-----
					if (ic == 0)
						sum = weight[oc_wchw] << FRAC_BIT; // set bias with int format
					else
						sum = 0;
					iy = mul(oy, stride) - pad;
					iy_w = mul(iy, input_fm_w); // calculate input image height

					ky_ww = 0;
					for (ky = 0; ky < weight_size.d2; ky++) // kernel height
					{
						ix = mul(ox, stride) - pad; // calculate input image width
						for (kx = 0; kx < weight_size.d3; kx++) // kernel width
						{
							// iy = ky + mul(oy, stride) - pad; // calculate input image height
							// ix = kx + mul(ox, stride) - pad; // calculate input image width

							if (iy >= 0 && iy < rd_size.d2 && ix >= 0 && ix < rd_size.d3) // ignore padding
								sum += mul(in[input_offset + ic_ihw + iy_w + ix], weight[oc_wchw + ky_ww + kx + 1]);
							++ix;
						}
						++iy;
						iy_w += input_fm_w;
						ky_ww += weight_size.d3;
					}
					if (ic == 0)
						// out[output_offset + oc_chw + (oy, conv_size.d3) + ox] = (short)(sum >> FRAC_BIT);
						out[output_offset + oc_chw + oy_cw + ox] = (short)(sum >> FRAC_BIT);
					else
						// out[output_offset + oc_chw + (oy, conv_size.d3) + ox] += (short)(sum >> FRAC_BIT);
						out[output_offset + oc_chw + oy_cw + ox] += (short)(sum >> FRAC_BIT);

				}
				oy_cw += conv_size.d3;
			}
			ic_ihw += input_size_hw;
		}
		oc_chw += conv_size_hw;
		oc_wchw += weight_size_chw;
	}
}

void pooling()
{
	short *out = (short *)addr.wr_addr; // address of array: output image

	unsigned output_offset = 0; // output image offset
	unsigned input_offset = 0;  // input image offset
	// unsigned input_offset = mul(mul(WR_SIZE_D0, WR_SIZE_D1), mul(WR_SIZE_D2, WR_SIZE_D3));  // input image offset

	unsigned input_fm_w = conv_size.d3; // input image width
	unsigned input_fm_h = conv_size.d2; // input image height

	unsigned pad = KERN_ATTR_POOL_PAD; // padding length
	unsigned pad_len = pad << 1;       // padding length * 2

	unsigned pad_w_test = conv_size.d3 - KERN_ATTR_POOL_KERN_SIZE; // width  - kernel_size, 22
	unsigned pad_h_test = conv_size.d2 - KERN_ATTR_POOL_KERN_SIZE; // height - kernel_size, 22

	unsigned pool_out_w = pad_w_test + pad_len; // output image width  = (width  - kern_size + pad_len)/stride + ~%stride + 1, 12
	unsigned pool_out_h = pad_h_test + pad_len; // output image height = (height - kern_size + pad_len)/stride + ~%stride + 1, 12

	unsigned stride = KERN_ATTR_POOL_STRIDE; // 2

	unsigned pad_w_test_remain = pad_w_test - mul(div(pad_w_test, stride), stride); // (width  - kernel_size) % stride
	unsigned pad_h_test_remain = pad_h_test - mul(div(pad_h_test, stride), stride); // (height - kernel_size) % stride

	pool_out_w = div(pool_out_w, stride); 
	pool_out_h = div(pool_out_h, stride); 
	pool_out_w++; // output image width  = (width  - kern_size + pad_len) / stride + 1
	pool_out_h++; // output image height = (height - kern_size + pad_len) / stride + 1

	// if no padding and (width  - kernel_size) % stride != 0 or (height - kernel_size) % stride != 0
	if ((!pad) && (pad_w_test_remain || pad_h_test_remain))
	{
		pool_out_w++; // take remainder into account
		pool_out_h++; // take remainder into account
	}

	//TODO: Please add your implementation here

	unsigned oc;  // output channel
	unsigned oy;  // output height
	unsigned ox;  // output width
	unsigned ky;  // kernel height
	unsigned kx;  // kernel width
	signed int iy;  // input height
	signed int ix;  // input width
	short temp; // the pixel dealing with
	short max; // max value of pooling for one pixel

	/*
		For each output channel, we check each pixel for convolution results.
		Set default max as
	*/

	const short input_size_hw = mul(input_fm_h, input_fm_w); // input image size: height * width
	const short output_size_hw = mul(pool_out_h, pool_out_w); // output image size: height * width

	int oc_ihw;
	int iy_iw;

	oc_ihw = 0;
	for (oc = 0; oc < wr_size.d1; oc++) // output channel
	{
		for (oy = 0; oy < pool_out_h; oy++) // output image height
		{
			for (ox = 0; ox < pool_out_w; ox++) // output image width
			{
				max = (MIN_SHORT);
				iy = mul(oy, stride) - pad;
				iy_iw = mul(iy, input_fm_w);
				for (ky = 0; ky < KERN_ATTR_POOL_KERN_SIZE; ky++) // kernel height
				{
					ix = mul(ox, stride) - pad;
					for (kx = 0; kx < KERN_ATTR_POOL_KERN_SIZE; kx++) // kernel width
					{
						// iy = ky + oy * stride - pad
						// ix = kx + ox * stride - pad
						// iy = ky + mul(oy, stride) - pad; // calculate input image height
						// ix = kx + mul(ox, stride) - pad; // calculate input image width
						if (iy >= 0 && iy < input_fm_h && ix >= 0 && ix < input_fm_w) // padding
							// if (max < (temp = out[mul(oc, input_size_hw) + mul(iy, input_fm_w) + ix]))
							if (max < (temp = out[input_offset + oc_ihw + iy_iw + ix]))
								max = temp;
						++ix;
					}
					++iy;
					iy_iw += input_fm_w;
				}
				// out[oc][oy][ox] = max
				out[output_offset + mul(oc, output_size_hw) + mul(oy, pool_out_w) + ox] = max;
			}
		}
		oc_ihw += input_size_hw;
	}
}

#ifdef USE_HW_ACCEL
void launch_hw_accel()
{
	volatile int* gpio_start = (void*)(GPIO_START_ADDR);
	volatile int* gpio_done = (void*)(GPIO_DONE_ADDR);

	//TODO: Please add your implementation here

	(*gpio_start) |= 1; // start hardware accelerator, set gpio_start[0], for the 32-bit variable
	while (!((*gpio_done) & 1)) // wait for finishing, check if (pio_start[0] == 0), for the 32-bit variable
		;
}
#endif

int comparing()
{
	char *out = (char *)addr.wr_addr;
	char *result = (char *)_binary_data_result_bin_start;

#ifdef USE_HW_ACCEL
	int count = (int)_binary_data_result_bin_size + 
		    (16 - WR_SIZE_D3) * 2 * WR_SIZE_D2 * WR_SIZE_D1;
#else
	int count = (int)_binary_data_result_bin_size;
#endif

	for (int i = 0, j = 0; i < count; i++)
	{
#ifdef USE_HW_ACCEL
		int alignment = i & 0x0000001f;
		if (alignment >= (WR_SIZE_D3 << 1))
			continue;
#endif
		if (*(out + i) != *(result + j))
		{
			printf("Failed! at address %x and %x with data %x and %x\n", out + i, result + j, *(out + i), *(result + j));
			return 1;
		}
		j++;
	}

	printf("Passed!\n");
	return 0;
}

int main()
{
	Result res;
	bench_prepare(&res); // clean everything, start timer

#ifdef USE_HW_ACCEL
	printf("Launching task...\n");
	launch_hw_accel();
#else
	printf("starting convolution\n");
	convolution();

	char *out = (char *)addr.wr_addr;
	printf("=================================\n");
	for (int i = 0; i < WR_SIZE_D1 * 24 * 24; i++)
	{
		printf("%04x ", (unsigned)(out[i]));
		if (i % 24 == 23)
			printf("\n");
		if (i % 576 == 575)
			printf("\n");
	}
	printf("=================================\n");

	printf("starting pooling\n");
	pooling();
#endif

	bench_done(&res); // collect results
	printf("Total cycles: %u%09u\n",            (unsigned)res.msec_hi, (unsigned)res.msec);
	printf("Memory load counts: %u\n",          (unsigned)res.mem_load_time);
	printf("Memory store counts: %u\n",         (unsigned)res.mem_store_time);
	printf("Instruction numbers: %u\n",         (unsigned)res.inst_num);
	printf("Memory load cycles: %u\n",          (unsigned)res.mem_load_cycle);
	printf("Memory store cycles: %u\n",         (unsigned)res.mem_store_cycle);
	printf("Instruction fetching cycles: %u\n", (unsigned)res.inst_fetch_cycle);
	printf("Total happened PC jump: %u\n",      (unsigned)res.jump_num);

	printf("Memory reference time: %u\n",       (unsigned)(res.mem_load_time + res.mem_store_time));
	printf("CPI: %u\n",                         (unsigned)div(res.msec, res.inst_num));

	int result = comparing();
	printf("benchmark finished\n");

	if (result == 0) {
		hit_good_trap();
	} else {
		nemu_assert(0);
	}

	return 0;
}

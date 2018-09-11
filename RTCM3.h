#pragma once
#include "Input.h"
#include "ProblemDef.h"
#include "DataStore.h"




#define buffer_size 100				//缓冲区大小定义，在函数lock_data内使用。表示一轮同步位捕捉的缓冲区大小。
#define bits_in_byte 8				//byte中的位数量。在dsp下，需要设置其为10。

#define bytes_in_rtcmword 5			//一个rtcm字中的字节数量。按照协议，其值为5。------------------------------协议规定
#define PseudorangeInvalid 1e5		//协议规定的伪距改正值不可用时的默认值。设置该值意味着弃用该卫星。---------协议规定

struct RTCMWord {					//RTCM 字。紧密结构，需要对齐成员。
	unsigned char data[bytes_in_rtcmword];	//五字节数据存储。
};


class RTCMDifferentialFileInput : public FileInput, public DifferentialInput, public RTCM3Input {
private:
	const int PRNINDEX[32] =
	{
		32,1,2,3,4,5,6,7,8,9,10,11,12,
		13,14,15,16,17,18,19,20,21,22,
		23,24,25,26,27,28,29,30,31
	};

	const unsigned char rtcm_bytes_litter_lim = 0x40;	//rctm字节的数值下界。 any_byte >= rtcm_bytes_litter_lim ----------协议规定
	const unsigned char rtcm_bytes_upper_lim = 0x7F;	//rctm字节的数值上界。 any_byte <= rtcm_bytes_upper_lim -----------协议规定
	const short type_rtk = 1;					//差分数据所对应的message_type. -----------------------------------协议规定
	const double PRC[2] = { 0.02,0.32 };		//伪距改正值乘常数  p = value * PRC[s] ----------------------------协议规定
	const double RRC[2] = { 0.002,0.032 };		//速度改正值乘常数  r = value * RRC[s] ----------------------------协议规定

protected:
	int mask;

	void get_rtcm_word(unsigned char * data, int offset, RTCMWord * out, int & mask)
	{
		unsigned char mask_a = (1 << (bits_in_byte - offset)) - 1;
		unsigned char mask_b = ~mask_a;
		for (int i = 0; i < bytes_in_rtcmword; i++)
			out->data[i] = ((data[i] & mask_a) << offset) | ((data[i + 1] & mask_b) >> (bits_in_byte - offset));

		mask = 0;
		mask |= (out->data[bytes_in_rtcmword - 1] & 0x20) >> 5;
		mask |= (out->data[bytes_in_rtcmword - 1] & 0x10) >> 3;
	}

	int rtcm_word_cypher(RTCMWord * word, int mask)
	{
		int tot = 0;
		int oppo_mask = mask & 1;
		for (int i = 0; i<bytes_in_rtcmword - 1; i++)
		{
			int temp = 0;
			temp |= (word->data[i] & 0b00100000) >> 5;
			temp |= (word->data[i] & 0b00010000) >> 3;
			temp |= (word->data[i] & 0b00001000) >> 1;
			temp |= (word->data[i] & 0b00000100) << 1;
			temp |= (word->data[i] & 0b00000010) << 3;
			temp |= (word->data[i] & 0b00000001) << 5;
			if (oppo_mask == 1)
			{
				temp = ~temp;
				temp &= 0x3F;
			}
			tot |= temp << (6 * (bytes_in_rtcmword - 2 - i));
		}

		int check_raw[6] = { 0 }, check_cnt[6] = { 0 };
		check_raw[5] = (word->data[bytes_in_rtcmword - 1] & 0b00100000) >> 5;
		check_raw[4] = (word->data[bytes_in_rtcmword - 1] & 0b00010000) >> 4;
		check_raw[3] = (word->data[bytes_in_rtcmword - 1] & 0b00001000) >> 3;
		check_raw[2] = (word->data[bytes_in_rtcmword - 1] & 0b00000100) >> 2;
		check_raw[1] = (word->data[bytes_in_rtcmword - 1] & 0b00000010) >> 1;
		check_raw[0] = (word->data[bytes_in_rtcmword - 1] & 0b00000001);

		int bit[26];
		for (int i = 0; i < 24; i++)
		{
			bit[i] = (tot & (int)pow(2, 24 - i - 1)) >> (24 - i - 1);
		}
		bit[24] = (mask & 2) >> 1; bit[25] = mask & 1;
		check_cnt[0] = bit[0] ^ bit[1] ^ bit[2] ^ bit[4] ^ bit[5] ^ bit[9] ^ bit[10] ^ bit[11] ^ bit[12] ^ bit[13] ^ bit[16] ^ bit[17] ^ bit[19] ^ bit[22] ^ bit[24];
		check_cnt[1] = bit[1] ^ bit[2] ^ bit[3] ^ bit[5] ^ bit[6] ^ bit[10] ^ bit[11] ^ bit[12] ^ bit[13] ^ bit[14] ^ bit[17] ^ bit[18] ^ bit[20] ^ bit[23] ^ bit[25];
		check_cnt[2] = bit[0] ^ bit[2] ^ bit[3] ^ bit[4] ^ bit[6] ^ bit[7] ^ bit[11] ^ bit[12] ^ bit[13] ^ bit[14] ^ bit[15] ^ bit[18] ^ bit[19] ^ bit[21] ^ bit[24];
		check_cnt[3] = bit[1] ^ bit[3] ^ bit[4] ^ bit[5] ^ bit[7] ^ bit[8] ^ bit[12] ^ bit[13] ^ bit[14] ^ bit[15] ^ bit[16] ^ bit[19] ^ bit[20] ^ bit[22] ^ bit[25];
		check_cnt[4] = bit[0] ^ bit[2] ^ bit[4] ^ bit[5] ^ bit[6] ^ bit[8] ^ bit[9] ^ bit[13] ^ bit[14] ^ bit[15] ^ bit[16] ^ bit[17] ^ bit[20] ^ bit[21] ^ bit[23] ^ bit[25];
		check_cnt[5] = bit[2] ^ bit[4] ^ bit[5] ^ bit[7] ^ bit[8] ^ bit[9] ^ bit[10] ^ bit[12] ^ bit[14] ^ bit[18] ^ bit[21] ^ bit[22] ^ bit[23] ^ bit[24];

		if (memcmp(check_cnt, check_raw, 6 * sizeof(int)) == 0)
		{
			return tot;
		}

		else {
			return -1;
		}
	}

	bool is_valid_header_rtcm_word(RTCMWord * word, int mask)
	{
		static unsigned char sync_flag[2][2] = { { 0x66,0x59 },{ 1,2 } };
		int oppo_mask = mask & 1;
		if (word->data[0] == sync_flag[0][oppo_mask])
			if ((word->data[1] & 3) == sync_flag[1][oppo_mask])
				return true;
		return false;
	}

	bool try_get_rtcm_word(unsigned char * data, int offset, RTCMWord * out, int & mask)
	{
		unsigned char mask_a = pow(2, bits_in_byte - offset) - 1;
		unsigned char mask_b = ~mask_a;
		for (int i = 0; i < bytes_in_rtcmword; i++)
		{
			out->data[i] = ((data[i] & mask_a) << offset) | ((data[i + 1] & mask_b) >> (bits_in_byte - offset));
			if (out->data[i] < rtcm_bytes_litter_lim || out->data[i] > rtcm_bytes_upper_lim)
				return false;
		}

		mask = 0;
		mask |= (out->data[bytes_in_rtcmword - 1] & 0x20) >> 5;
		mask |= (out->data[bytes_in_rtcmword - 1] & 0x10) >> 3;

		return true;
	}

	bool lock_data(FILE * fp, int & mask, int & sync_j, int & frame_count)
	{
		unsigned char temp_buffer[buffer_size];
		RTCMWord words[2];
		int ori = ftell(fp);
		int mask_ori = mask;
		int mask_set = mask;
		while (!feof(fp))
		{
			fread(temp_buffer, 1, buffer_size, fp);

			for (int i = 0; i<buffer_size - 10; i++)
			{
				for (int j = 0; j < bits_in_byte; j++)
				{
					if (try_get_rtcm_word(temp_buffer + i, j, words, mask))
					{
						if (is_valid_header_rtcm_word(words, mask_ori))
						{
							mask_set = mask_ori;
							int word1 = rtcm_word_cypher(words, mask_ori);

							mask_ori = mask;

							get_rtcm_word(temp_buffer + i + 5, j, words + 1, mask);
							int word2 = rtcm_word_cypher(words + 1, mask_ori);
							frame_count = (word1 != -1 && word2 != -1) ? ((word2 & 0x0000F8) >> 3) + 2 : 2;

							int mes_id = (word1 & 0x00FC00) >> 10;
							sync_j = j;
							if (i == 0)fseek(fp, ori, SEEK_SET);
							else fseek(fp, i - buffer_size, SEEK_CUR);
							mask = mask_set;

							return true;
						}
						else {
							mask_ori = mask;
						}
					}
				}
			}
		}
		throw FILE_END_REACHED;
	}

	bool get_rtcm_message(unsigned char * data, int * values, int & mask, int offset, int frame)
	{
		for (int i = 0; i < frame; i++)
		{
			int mask_ori = mask;
			RTCMWord temp;
			get_rtcm_word(data + bytes_in_rtcmword * i, offset, &temp, mask);
			int tempdata = rtcm_word_cypher(&temp, mask_ori);
			if (tempdata == -1)return false;
			values[i] = tempdata;
		}
		return true;
	}

	void parse_header(int * value, RTCMMessageHeader * header)
	{
		header->message_type = (value[0] & 0x00FC00) >> 10;
		header->station_id = (value[0] & 0x0003FF);
		header->z_count = ((short)((value[1] & 0xFFF800) >> 11)) * 0.6;
		header->nos = (value[1] & 0x000700) >> 8;
		header->mslength = (value[1] & 0x0000F8) >> 3;
		header->health = (value[1] & 0x000007);
	}

	void parse_blocks(int * value, RTCMMessageFrame * out, int total)
	{
		out[0].s = (value[0] & 0x800000) >> 23;
		out[0].udre = (value[0] & 0x600000) >> 21;
		int prn = (short)((value[0] & 0x1F0000) >> 16);
		out[0].prn = PRNINDEX[prn];
		int psrcorrection = (short)(((value[0]) & 0x00FFFF));
		out[0].psrcorrection = psrcorrection != PseudorangeInvalid ? psrcorrection * PRC[out[0].s] : PseudorangeInvalid;
		out[0].rvcorrection = ((short)((value[1] & 0xFF0000) >> 16)) * RRC[out[0].s];
		out[0].ageofdata = (value[1] & 0x00FF00) >> 8;

		if (total == 1)return;
		out[1].s = (value[1] & 0x000080) >> 7;
		out[1].udre = (value[1] & 0x000060) >> 5;
		prn = (value[1] & 0x00001F);
		out[1].prn = PRNINDEX[prn];
		psrcorrection = (short)((value[2] & 0xFFFF00) >> 8);
		out[1].psrcorrection = psrcorrection != PseudorangeInvalid ? psrcorrection * PRC[out[1].s] : PseudorangeInvalid;
		out[1].rvcorrection = ((short)(value[2] & 0x0000FF)) * RRC[out[1].s];
		out[1].ageofdata = (value[3] & 0xFF0000) >> 16;

		if (total == 2)return;
		out[2].s = (value[3] & 0x008000) >> 15;
		out[2].udre = (value[3] & 0x006000) >> 13;
		prn = (value[3] & 0x001F00) >> 8;
		out[2].prn = PRNINDEX[prn];
		psrcorrection = (short)(((value[3] & 0x0000FF) << 8) | ((value[4] & 0xFF0000) >> 16));
		out[2].psrcorrection = psrcorrection != PseudorangeInvalid ? psrcorrection * PRC[out[2].s] : PseudorangeInvalid;
		out[2].rvcorrection = ((short)((value[4] & 0x00FF00) >> 8)) * RRC[out[2].s];
		out[2].ageofdata = (value[4] & 0x0000FF);
	}
	void decode(RTCMDiffMessage & message, int * value, int length)
	{
		int num_of_block_five = length / 5;
		int num_of_block_four = (length - (num_of_block_five * 5)) / 4;
		int num_of_block_two = (length - (num_of_block_five * 5)) / 2 - num_of_block_four * 2;


		parse_header(value, &message.header);
		if (message.header.message_type == type_rtk)
		{
			message.satellite_amount = num_of_block_five * 3 + num_of_block_four * 2 + num_of_block_two;
			message.frames = (RTCMMessageFrame*)malloc(sizeof(RTCMMessageFrame) * message.satellite_amount);

			for (int i = 0; i<num_of_block_five; i++)
			{
				parse_blocks(value + 2 + i * 5, message.frames + i * 3, 3);
			}
			for (int i = 0; i<num_of_block_four; i++)
			{
				parse_blocks(value + 2 + num_of_block_five * 5 + i * 4, message.frames + num_of_block_five * 3 + i * 2, 2);
			}
			for (int i = 0; i<num_of_block_two; i++)
			{
				parse_blocks(value + 2 + num_of_block_five * 5 + num_of_block_four * 4 + i * 2, message.frames + num_of_block_five * 3 + num_of_block_four * 2 + i * 1, 1);
			}

			for (int i = 0; i < message.satellite_amount; i++)
			{
				message.frames[i].tick_time = message.header.z_count;
			}
		}
	}

	bool message_valid_check(RTCMDiffMessage & message)
	{
		if (message.header.health != 0 && message.header.health != 1 && message.header.health != 6)
			return false;
		if (message.header.message_type != 1)
			return false;
		if (message.header.z_count > 3600 || message.header.z_count < 0)
			return false;

		for (int i = 0; i < message.satellite_amount; i++)
		{
			if (message.frames[i].prn > 32 || message.frames[i].prn <= 0)
				return false;
			if (message.frames[i].s != 0 && message.frames[i].s != 1)
				return false;
			if (message.frames[i].udre >= 4 || message.frames[i].udre < 0)
				return false;
			if (message.frames[i].psrcorrection <= -50 || message.frames[i].psrcorrection > 5)
				return false;
			if (message.frames[i].rvcorrection >= 1 || message.frames[i].rvcorrection <= -1)
				return false;
		}
		return true;
	}

public:
	RTCMDifferentialFileInput(wstring fn)
	{
		media = InputMedia::IN_DISK;
		protocol = InputProtocol::IN_RTCM3;
		usage = InputUsage::IN_DIFFERENTIAL;
		filename = fn;
		mask = 1;

		fp = _wfopen(filename.c_str(), L"r");

	}

	bool try_once(RTCMDiffMessage & message)
	{
		int sync_j = 0;
		int frame_count = 0;
		if (lock_data(fp, mask, sync_j, frame_count))
		{
			unsigned char * buffer = (unsigned char*)alloca(frame_count * 5);
			int * values = (int*)alloca(sizeof(int) * frame_count);

			fread(buffer, frame_count, 5, fp);
			if (get_rtcm_message(buffer, values, mask, sync_j, frame_count))
			{
				decode(message, values, frame_count - 2);
				return message_valid_check(message);
			}
			else return false;

		}
		return false;
	}

};
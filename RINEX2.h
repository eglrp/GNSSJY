#pragma once
#include <ostream>
#include <fstream>

#include "DataStore.h"
#include "Solver.h"
#include "Input.h"
#include "Output.h"
#include "ProblemDef.h"


char symbol_of_rinex_obs[MAX_OBSER_TYPE][3]
{
	{"L1"},{"L2"},{"P1"},{"P2"},{"C1"},{"C2"},{"S1"},{"S2"},{"D1"},
};

struct RinexNavFileHeader: public RinexFileHeader {

};

struct RinexObsFileHeader : public StationInfo, public RinexFileHeader {

	// INTERVAL
	double interval;

	// # / TYPES OF OBSERV
	int obs_number;
	unsigned char type_of_obs[MAX_OBSER_TYPE];

	// TIME OF FIRST OBS
	char time_of_first[61];

	// TIME OF LAST OBS
	char time_of_last[61];

	// MARKER NAME
	char marker_name[61];

	// MARKER NUMBER
	char marker_number[21];
};

struct RinexIonFileHeader : public RinexFileHeader{

	// EPOCH OF FIRST MAP
	char time_of_first[61];

	// EPOCH OF LAST MAP
	char time_of_last[61];

	// INTERVAL
	double interval; // 0 for variable intervals

	// # OF MAPS IN FILE
	int maps;

	// MAPPING FUNCTION
	char mapping_function[5]; // 'NONE' for no mf, 'COSZ' for 1/cos(Z), 'QFAC' for Q-factor

	// ELEVATION CUTOFF
	double elevation_cutoff; // 0.0 for unknown, 90.0 for altimetry.

	// OBSERVABLES USED 
	char observables_used[61];

	// # OF STATIONS
	int of_stations;

	// # OF SATELLITES
	int of_satellites;

	// BASE RADIUS
	double base_radius; // for km

	// MAP DIMENSION
	int map_dimention;

	// HGT1 / HGT2 / DHGT
	double HGT12D[3];

	// LAT1 / LAT2 / DLAT
	double LAT12D[3];

	// LON1 / LON2 / DLON
	double LON12D[3];

	// EXPONENT
	int exponent;
};



class RINEXIonosphereFileInput: public FileInput, public IonosphereInput, public RINEX2Input{
private:
	RinexIonFileHeader header;
	//char ion_buffer[7];
protected:
	virtual int parse_header_line(const char * content, const char * title)
	{
		if(!strncmp(title, "RINEX VERSION / TYPE", 20))
		{
			sscanf(content, "%lf", &header.rinex_version);
			if(header.rinex_version >= 3)return -1;

			strncpy(header.file_type,        content + 20, 19);
			if(strcmp(header.file_type, "N: GPS NAV DATA    ")) return -1;

			strncpy(header.satellite_system, content + 39, 19);
		}
		else if(!strncmp(title, "PGM / RUN BY / DATE", 19))
		{
			strncpy(header.name_of_pgm,      content     , 20);
			strncpy(header.name_of_agency,   content + 20, 20);
			strncpy(header.data_of_create,   content + 40, 20);
		}
		else if(!strncmp(title, "EPOCH OF FIRST MAP", 18))
		{
			strncpy(header.time_of_first,    content, 60);
		}
		else if(!strncmp(title, "EPOCH OF LAST MAP", 17))
		{
			strncpy(header.time_of_last,     content, 60);
		}
		else if(!strncmp(title, "INTERVAL", 8))
		{
			sscanf(content, "%lf", &header.interval);
		}
		else if(!strncmp(title, "# OF MAPS IN FILE", 17))
		{
			sscanf(content, "%d", &header.maps);
		}
		else if(!strncmp(title, "MAPPING FUNCTION", 16))
		{
			strncpy(header.mapping_function,  content + 2, 4);
		}
		else if(!strncmp(title, "ELEVATION CUTOFF", 16))
		{
			sscanf(content, "%lf", &header.elevation_cutoff);
		}
		else if(!strncmp(title, "OBSERVABLES USED", 16))
		{
			strncpy(header.observables_used,  content, 60);
		}
		else if(!strncmp(title, "# OF STATIONS", 13))
		{
			sscanf(content, "%d", &header.of_stations);
		}
		else if(!strncmp(title, "# OF SATELLITES", 15))
		{
			sscanf(content, "%d", &header.of_satellites);
		}
		else if(!strncmp(title, "BASE RADIUS", 11))
		{
			sscanf(content, "%lf", &header.base_radius);
		}
		else if(!strncmp(title, "HGT1 / HGT2 / DHGT", 18))
		{
			sscanf(content, "%lf%lf%lf", 
				&header.HGT12D[0], &header.HGT12D[1], &header.HGT12D[2]);
		}
		else if(!strncmp(title, "LAT1 / LAT2 / DLAT", 18))
		{
			sscanf(content, "%lf%lf%lf", 
				&header.LAT12D[0], &header.LAT12D[1], &header.LAT12D[2]);
		}
		else if(!strncmp(title, "LON1 / LON2 / DLON", 18))
		{
			sscanf(content, "%lf%lf%lf", 
				&header.LON12D[0], &header.LON12D[1], &header.LON12D[2]);
		}
		else if(!strncmp(title, "END OF HEADER", 13))
		{
			return 1;
		}
		return 0;
	}

	virtual bool parse_header()
	{
		if (!fp) { any_error = true; return false; }

		while(true){
			if(feof(fp)) {
				any_error = true;
				throw FILE_FORMAT_NOT_CORRECT;
			}
			fgets(line_buffer, 61, fp);
			fgets(line_buffer + 61, 30, fp);
			line_buffer[60 + strlen(line_buffer + 61)] = '\0';

			int ret = parse_header_line(line_buffer, line_buffer + 61);
			
			if(ret == 1) break; // means the end of reading
			else if(ret == -1) {
				any_error = true;
				throw FILE_TYPE_NOT_CORRECT;
			}
		}

		return true;
	}
	// double extract_double(const char * pointee, int length)
	// {
	// 	if(strlen(pointee) < length) return 0;
	// 	strncpy(ion_buffer, pointee, length);
	// 	return atof(ion_buffer);
	// }
public:
	virtual bool is_valid()
	{
		return !any_error;
	}
	RINEXIonosphereFileInput(wstring fn, TECMap * map) {
		media = InputMedia::IN_DISK;
		protocol = InputProtocol::IN_RINEX2;
		usage = InputUsage::IN_IONOSPHERE;
		filename = fn;

		fp = _wfopen(fn.c_str(), L"r");
		if(fp == NULL) throw FILE_NOT_FOUND;

		if(!parse_header()){
			throw UNEXPECTED;
		}

		int rows = (int)round((header.LAT12D[1] - header.LAT12D[0]) / header.LAT12D[2]) + 1;
		int cols = (int)round((header.LON12D[1] - header.LON12D[0]) / header.LON12D[2]) + 1;
		map->data = malloc_mat(rows, cols);
		map->row_step = header.LAT12D[2];
		map->col_step = header.LON12D[2];
		map->row_start = header.LAT12D[0];
		map->col_start = header.LON12D[0];
		map->fresh_time = UTC();
	}

	virtual bool try_once(TECMap * map, UTC * time)
	{
		bool renewed = false;
		if(time->minus(&map->fresh_time) >= header.interval)
		{
			if(feof(fp)) {
				any_error = true;
				throw FILE_FORMAT_NOT_CORRECT;
			}

			fgets(line_buffer, 100, fp); // START OF TEC MAP
			fgets(line_buffer, 100, fp); // EPOCH OF CURRENT MAP
			
			double sec;
			sscanf(line_buffer, "%d%d%d%d%d%lf",
				&map->fresh_time.year,
				&map->fresh_time.month,
				&map->fresh_time.date,
				&map->fresh_time.hour,
				&map->fresh_time.minute,
				&sec
			);
			map->fresh_time.sec = (int)round(sec);
			map->fresh_time.year -= map->fresh_time.year >= 2000 ? 2000 : 1900;

			fgets(line_buffer, 100, fp); // LAT/LON1/LON2/DLON/H

			for(int i = 0; i < map->data->rows; i ++)
			{
				for(int j = 0;j < map->data->cols; j++)
				{
					fscanf(fp, "%lf", &map->data->data[i][j]);
				}
				fscanf(fp, "\n");
				fgets(line_buffer, 100, fp); // LAT/LON1/LON2/DLON/H and // END OF TEC MAP
			}
			map->check(true);
		}

		return true;
	}
};

class RINEX2NavigationFileInput: public	FileInput, public NavigationInput, public RINEX2Input{
private:
	
	RinexNavFileHeader header;
	UTC next_navi_time;
	
	char nav_buffer[20];
	int next_prn;

protected:
	virtual int parse_header_line(const char * content, const char * title)
	{
		if(!strncmp(title, "RINEX VERSION / TYPE", 20))
		{
			sscanf(content, "%lf", &header.rinex_version);
			if(header.rinex_version >= 3)return -1;

			strncpy(header.file_type,        content + 20, 19);
			if(strcmp(header.file_type, "N: GPS NAV DATA    ")) return -1;

			strncpy(header.satellite_system, content + 39, 19);
		}
		else if(!strncmp(title, "PGM / RUN BY / DATE", 19))
		{
			strncpy(header.name_of_pgm,      content     , 20);
			strncpy(header.name_of_agency,   content + 20, 20);
			strncpy(header.data_of_create,   content + 40, 20);
		}
		else if(!strncmp(title, "END OF HEADER", 13))
		{
			return 1;
		}
		return 0;
	}

	virtual bool parse_header()
	{
		if (!fp) { any_error = true; return false; }

		while(true){
			if(feof(fp)) {
				any_error = true;
				throw FILE_FORMAT_NOT_CORRECT;
			}
			fgets(line_buffer, 61, fp);
			fgets(line_buffer + 61, 30, fp);
			line_buffer[60 + strlen(line_buffer + 61)] = '\0';

			int ret = parse_header_line(line_buffer, line_buffer + 61);
			
			if(ret == 1) break; // means the end of reading
			else if(ret == -1) {
				any_error = true;
				throw FILE_TYPE_NOT_CORRECT;
			}
		}

		return true;
	}

	void grab_time()
	{
		if (feof(fp)) throw FILE_END_REACHED;
		double sec;
		fscanf(fp, "%d%d%d%d%d%d%lf", 
			&next_prn,
			&next_navi_time.year,
			&next_navi_time.month,
			&next_navi_time.date,
			&next_navi_time.hour,
			&next_navi_time.minute,
			&sec
		);
		next_navi_time.sec = (int)round(sec);
	}

	double extract_double(const char * pointee)
	{
		if(strlen(pointee) < 19) return 0;
		strncpy(nav_buffer, pointee, 19);
		nav_buffer[15] = 'E';
		return atof(nav_buffer);
	}
public:

	RINEX2NavigationFileInput(wstring fn) {
		media = InputMedia::IN_DISK;
		protocol = InputProtocol::IN_RINEX2;
		usage = InputUsage::IN_NAVIGATION;
		filename = fn;

		fp = _wfopen(fn.c_str(), L"r");
		if(fp == NULL) throw FILE_NOT_FOUND;

		if(!parse_header()){
			throw UNEXPECTED;
		}

		grab_time();
	}

	virtual bool try_once(Broadcast * nav, UTC * time)
	{
		bool renewed = false;

		while(time->larger_than(next_navi_time))
		{
			renewed = true;

			// PRN / EPOCH / SV CLK
			if (fgets(line_buffer, 90, fp) == NULL) throw FILE_END_REACHED;
			nav[next_prn].toc = GPSTime(next_navi_time);
			nav[next_prn].sv_clock_bias       = extract_double(line_buffer     );
			nav[next_prn].sv_clock_drift      = extract_double(line_buffer + 19);
			nav[next_prn].sv_clock_drift_rate = extract_double(line_buffer + 38);

			// BROADCAST ORBIT - 1
			fgets(line_buffer, 4, fp);
			fgets(line_buffer, 90, fp);
			nav[next_prn].idoe_issue_of_data  = extract_double(line_buffer     );
			nav[next_prn].crs                 = extract_double(line_buffer + 19);
			nav[next_prn].delta_n             = extract_double(line_buffer + 38);	
			nav[next_prn].m0                  = extract_double(line_buffer + 57);

			// BROADCAST ORBIT - 2
			fgets(line_buffer, 4, fp);
			fgets(line_buffer, 90, fp);
			nav[next_prn].cuc                 = extract_double(line_buffer     );
			nav[next_prn].eccentricity        = extract_double(line_buffer + 19);
			nav[next_prn].cus                 = extract_double(line_buffer + 38);	
			nav[next_prn].sqrt_a              = extract_double(line_buffer + 57);							
		
			// BROADCAST ORBIT - 3
			fgets(line_buffer, 4, fp);
			fgets(line_buffer, 90, fp);
			nav[next_prn].toe                 = extract_double(line_buffer     );
			nav[next_prn].cic                 = extract_double(line_buffer + 19);
			nav[next_prn].OMEGA               = extract_double(line_buffer + 38);	
			nav[next_prn].cis                 = extract_double(line_buffer + 57);	

			// BROADCAST ORBIT - 4
			fgets(line_buffer, 4, fp);
			fgets(line_buffer, 90, fp);
			nav[next_prn].i0                  = extract_double(line_buffer     );
			nav[next_prn].crc                 = extract_double(line_buffer + 19);
			nav[next_prn].omega               = extract_double(line_buffer + 38);	
			nav[next_prn].omega_dot           = extract_double(line_buffer + 57);			

			// BROADCAST ORBIT - 5
			fgets(line_buffer, 4, fp);
			fgets(line_buffer, 90, fp);
			nav[next_prn].idot                = extract_double(line_buffer     );
			nav[next_prn].codes_on_l2         = extract_double(line_buffer + 19);
			nav[next_prn].gpsweek             = extract_double(line_buffer + 38);	
			nav[next_prn].l2_pdata_flag       = extract_double(line_buffer + 57);

			// BROADCAST ORBIT - 6
			fgets(line_buffer, 4, fp);
			fgets(line_buffer, 90, fp);
			nav[next_prn].sv_accuracy         = extract_double(line_buffer     );
			nav[next_prn].sv_health           = extract_double(line_buffer + 19);
			nav[next_prn].tgd                 = extract_double(line_buffer + 38);	
			nav[next_prn].iodc_issue_of_data  = extract_double(line_buffer + 57);			

			// BROADCAST ORBIT - 7
			fgets(line_buffer, 4, fp);
			fgets(line_buffer, 90, fp);
			nav[next_prn].trans_time          = extract_double(line_buffer     );
			nav[next_prn].fit_interval        = extract_double(line_buffer + 19);

			nav[next_prn].check(true);

			grab_time();
		}

		return renewed;
	}

	//virtual wstring description(){
	//	std::wostringstream w;
	//	w << L"RINEX2ObservationFileInput filename: " << filename << endl;
	//	return w.str();
	//}


	virtual bool is_valid()
	{
		return !any_error;
	}

};

class RINEX2ObservationFileInput: public FileInput, public ObservationInput, public RINEX2Input{
private:
	RinexObsFileHeader header;

	// temperory usage
	char sat_types[GNSS_SATELLITE_AMOUNT];
	int  sat_prns[GNSS_SATELLITE_AMOUNT];
	int health = -1;
	int sat_num = -1;
	char obs_buffer[15];

protected:


	// returns 0 for normal success, 1 for END OF HEADER symbol, -1 for invalid(something went wrong)
	virtual int parse_header_line(const char * content, const char * title)
	{
		if(!strncmp(title, "RINEX VERSION / TYPE", 20))
		{
			sscanf(content, "%lf", &header.rinex_version);
			if(header.rinex_version >= 3)return -1;

			strncpy(header.file_type,        content + 20, 19);
			if(strcmp(header.file_type, "OBSERVATION DATA   ")) return -1;

			strncpy(header.satellite_system, content + 39, 19);
		}
		else if(!strncmp(title, "PGM / RUN BY / DATE", 19))
		{
			strncpy(header.name_of_pgm,      content     , 20);
			strncpy(header.name_of_agency,   content + 20, 20);
			strncpy(header.data_of_create,   content + 40, 20);
		}
		else if(!strncmp(title, "INTERVAL", 8))
		{
			sscanf(content, "%lf", &header.interval);
		}
		else if(!strncmp(title, "# / TYPES OF OBSERV", 19))
		{
			char temp[3] = "";
			sscanf(content, "%d", &header.obs_number);
			for(int i = 0; i < header.obs_number; i++)
			{
				int offset = 6 * (i + 1) + 4;
				strncpy(temp,   content + offset, 2);
				for(int j = 0; j < MAX_OBSER_TYPE; j++)
					if(strcmp(temp, symbol_of_rinex_obs[j]) == 0) header.type_of_obs[i] = j;
			}
		}
		else if(!strncmp(title, "APPROX POSITION XYZ", 19))
		{
			sscanf(content, "%lf%lf%lf", &header.approx_position.X, &header.approx_position.Y, &header.approx_position.Z);
		}
		else if(!strncmp(title, "ANTENNA: DELTA H/E/N", 20))
		{
			sscanf(content, "%lf%lf%lf", &header.antenna_delta.U, &header.antenna_delta.E, &header.antenna_delta.N);
		}
		else if(!strncmp(title, "TIME OF FIRST OBS", 17))
		{
			strncpy(header.time_of_first,    content, 60);
		}
		else if(!strncmp(title, "TIME OF LAST OBS", 16))
		{
			strncpy(header.time_of_last,     content, 60);
		}
		else if(!strncmp(title, "MARKER NAME", 11))
		{
			strncpy(header.marker_name,      content, 60);
		}
		else if(!strncmp(title, "MARKER NUMBER", 13))
		{
			strncpy(header.marker_number,    content, 20);
		}
		else if(!strncmp(title, "END OF HEADER", 13))
		{
			return 1;
		}
		return 0;
	}

	virtual bool parse_header()
	{
		if (!fp) { any_error = true; return false; }

		while(true){
			if(feof(fp)) {
				any_error = true;
				throw FILE_FORMAT_NOT_CORRECT;
			}
			fgets(line_buffer, 61, fp);
			fgets(line_buffer + 61, 30, fp);
			line_buffer[60 + strlen(line_buffer + 61)] = '\0';
			int ret = parse_header_line(line_buffer, line_buffer + 61);
			
			if(ret == 1) break; // means the end of reading
			else if(ret == -1) {
				any_error = true;
				throw FILE_TYPE_NOT_CORRECT;
			}
		}

		return true;
	}

public:
	RINEX2ObservationFileInput(wstring fn){
		media    = InputMedia::IN_DISK;
		protocol = InputProtocol::IN_RINEX2;
		usage    = InputUsage::IN_OBSERVATION;
		filename = fn;

		fp = _wfopen(fn.c_str(), L"r");
		if(fp == NULL) throw FILE_NOT_FOUND;

		if(!parse_header()){
			throw UNEXPECTED;
		}
	}

	double get_interval()
	{
		return header.interval;
	}
	//virtual wstring description(){
	//	std::wostringstream w;
	//	w << L"RINEX2ObservationFileInput filename: " << filename << endl;
	//	return w.str();
	//}

	StationInfo * get_sta()
	{
		return (StationInfo*) &header;
	}

	virtual bool get_once(Observation * obs, UTC * time)
	{

		if(feof(fp))throw FILE_END_REACHED;

		for(int i = 0; i< GNSS_SATELLITE_AMOUNT; i++) 
			obs[i].check(false);

		// currently we can just use GPS satellites.
		// parse the time & satellite table.

		if (fgets(line_buffer, 90, fp) == NULL) throw FILE_END_REACHED;

		double sec = 0;
		sscanf(line_buffer, "%d%d%d%d%d%lf", &time->year, &time->month, &time->date, &time->hour, &time->minute, &sec);
		time->sec = (int)round(sec);

		sscanf(line_buffer + 26, "%d%d", &health, &sat_num);
		if(health != 0)return false;

		int last_line_sat = 0;
		for(int i = 0; i < sat_num; i++)
		{
			int offset = 32 + (i - last_line_sat) * 3;
			if (strlen(line_buffer + offset) < 3)
			{
				fgets(line_buffer, 90, fp);
				last_line_sat = i;
				i = i - 1;
				continue;
			}
			sscanf(line_buffer + offset, "%c%2d", &sat_types[i], &sat_prns[i]);
			if (sat_types[i] == 'R')
			{
				sat_prns[i] += 50;
			}
			else if (sat_types[i] == 'C') {
				sat_prns[i] += 100;
			}
		}

		// parse the data sheet
		int first_line_num = header.obs_number > 5 ? 5 : header.obs_number;
		for(int i = 0;i < sat_num; i++)
		{
			fgets(line_buffer, 90, fp);
			//if (ceil(strlen(line_buffer) / 16.0) != first_line_num) continue;
			Observation & aim = obs[sat_prns[i]];

			// clear the last epoch data
			for(int j = 0; j < MAX_OBSER_TYPE; j++)
				aim.values[j] = -1;

			for(int j = 0; j < first_line_num; j++)
			{
				unsigned int offset = j * 16;
				int type = header.type_of_obs[j];
				if (strlen(line_buffer + offset) < 14U || strlen(line_buffer) < offset) {
					aim.values[type] = OBS_DISABLED;
					continue;
				}

				strncpy(obs_buffer, line_buffer + offset, 14);
				aim.values[type] = atof(obs_buffer);
				if (aim.values[type] == 0)aim.values[type] = OBS_DISABLED;

				if (strlen(line_buffer + offset + 14) >= 2) {
					char lli = line_buffer[offset + 14];
					char ss = line_buffer[offset + 15];
					aim.LLI[type] = lli > '0' ? lli - '0' : 0;
					aim.signal_strength[type] = ss - '0' ? ss - '0' : 0;
				}
				else {
					aim.LLI[type] = 0;
					aim.signal_strength[type] = 0;
				}
			}

			// the second line.
			if(header.obs_number > 5)
			{
				fgets(line_buffer, 90, fp);
				for(int j = 0; j < header.obs_number - 5; j++)
				{
					unsigned int offset = j * 16;
					int type = header.type_of_obs[j + 5];

					if (strlen(line_buffer + offset) < 14U || strlen(line_buffer) < offset) {
						aim.values[type] = OBS_DISABLED;
						continue;
					}

					strncpy(obs_buffer, line_buffer + offset, 14);
					aim.values[type] = atof(obs_buffer);
					if (aim.values[type] == 0)aim.values[type] = OBS_DISABLED;

					if (strlen(line_buffer + offset + 14) >= 2) {
						char lli = line_buffer[offset + 14];
						char ss = line_buffer[offset + 15];
						aim.LLI[type] = lli > '0' ? lli - '0' : 0;
						aim.signal_strength[type] = ss - '0' ? ss - '0' : 0;
					}
					else {
						aim.LLI[type] = 0;
						aim.signal_strength[type] = 0;
					}
				}				
			}

			obs[sat_prns[i]].check(true);
		}
		return true;
	}

	virtual bool is_valid()
	{
		return !any_error;
	}
};




#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// x = linspace(0,length(a),length(a));plotyy(x,a(:,19),x,a(:,24));

static FILE *fp;

typedef struct __attribute__((packed))
{
	uint32_t iTOW; // ms, GPS time of week of the navigation epoch
	uint16_t year; // year (UTC)
	uint8_t month; // month, range 1..12 (UTC)
	uint8_t day;   // day of month, range 1..31 (UTC)
	uint8_t hour;  // hour of day, range 0..23 (UTC)
	uint8_t min;   // minute of hour, range 0..59 (UTC)
	uint8_t sec;   // seconds of minute, range 0..60 (UTC)
	uint8_t valid; // 4 // validity flags, 0 - valid_date, 1 - valid_time, 2 - fully_resolved, 3 - valid_mag
	uint32_t tAcc; // ns, time accuracy estimate (UTC)
	int32_t nano;  // ns, fraction of second, range -1e9 .. 1e9 (UTC)

	uint8_t fixType;	  // 0 - no_fix, 1 - dead reckoning only, 2 - 2D, 3 - 3D, 4 - GNSS + dead reckoning combined, 5 - time only fix
	uint8_t flags;		  //
	uint8_t flags2;		  //
	uint8_t numSV;		  // number of satellites used in Nav Solution
	int32_t lon;		  // 1e-7 deg, longitude
	int32_t lat;		  // 1e-7 deg, latitude
	int32_t height;		  // mm, height above ellipsoid
	int32_t hMSL;		  // mm, height above mean sea level
	uint32_t hAcc;		  // mm, horizontal accuracy estimate
	uint32_t vAcc;		  // mm, vertical accuracy estimate
	int32_t velN;		  // mm/s, NED north velocity
	int32_t velE;		  // mm/s, NED east velocity
	int32_t velD;		  // mm/s, NED down velocity
	int32_t gSpeed;		  // mm/s, ground Speed (2-D)
	int32_t headMot;	  // 1e-5 deg, heading of motion (2-D)
	uint32_t sAcc;		  // mm/s, speed accuracy estimate
	uint32_t headAcc;	  // 1e-5 deg, heading accuracy estimate (both motion & vehicle)
	uint16_t pDOP;		  // 0.01, position DOP
	uint8_t reserved1[6]; //
	int32_t headVeh;	  // Heading of vehicle (2-D) [deg / 1e-5]
	int16_t magDec;		  // Magnetic declination [deg / 1e-2]
	uint16_t magAcc;	  // Magnetic declination accuracy [deg / 1e-2]
} NAV_PVT_t;

struct
{
	uint32_t t;
	int16_t accel_raw[3], gyro_raw[3];
	NAV_PVT_t pvt;
} smpl;

static size_t file_len(void)
{
	fseek(fp, 0, SEEK_END);
	size_t s = (size_t)ftell(fp);
	rewind(fp);
	return s;
}

FILE *fwgpx;

static void gpx_start(const char *fn, NAV_PVT_t *pvt)
{
	fwgpx = fopen(fn, "wb");
	fprintf(fwgpx, "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n"
				   "<gpx xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\" "
				   "xsi:schemaLocation=\"http://www.topografix.com/GPX/1/1 http://www.topografix.com/GPX/1/1/gpx.xsd\" "
				   "creator=\"StravaGPX\" version=\"1.1\" xmlns=\"http://www.topografix.com/GPX/1/1\">\n"
				   " <metadata>\n  <time>%04d-%02d-%02dT%02d:%02d:%02dZ</time>\n"
				   " </metadata>\n <trk>\n   <name>%s</name>\n   <type>walking</type>\n  <trkseg>\n",
			pvt->year, pvt->month, pvt->day, pvt->hour, pvt->min, pvt->sec, fn);
}

static void gpx_data(NAV_PVT_t *pvt)
{
	fprintf(fwgpx, "    <trkpt lat=\"%.7f\" lon=\"%.7f\">\n"
				   "     <ele>%.3f</ele>\n"
				   "     <time>%04d-%02d-%02dT%02d:%02d:%02dZ</time>\n"
				   "    </trkpt>\n",
			(double)smpl.pvt.lat * 1e-7, (double)smpl.pvt.lon * 1e-7, (double)smpl.pvt.height * 1e-3,
			pvt->year, pvt->month, pvt->day, pvt->hour, pvt->min, pvt->sec);
}

static void gpx_end(void)
{
	fprintf(fwgpx, "  </trkseg>\n </trk>\n</gpx>\n");
	fclose(fwgpx);
}

int main(int argc, char *argv[])
{
	if(argc < 1)
	{
		fprintf(stderr, "Usage: %s <filename>\n", argv[0]);
		return EXIT_FAILURE;
	}

	for(uint32_t i = 1; i < argc; i++)
	{
		const char *filename = argv[i];
		fp = fopen(filename, "rb");
		if(!fp)
		{
			perror("Error opening file");
			return EXIT_FAILURE;
		}

		size_t fl = file_len();
		size_t cnt = fl / sizeof(smpl);
		printf("File \"%s\" size: %8d | Samples: %d\n", filename, fl, cnt);
		if((fl % sizeof(smpl)) != 0)
		{
			printf("Wrong file size!\n");
			return EXIT_FAILURE;
		}

		size_t offset = 0;

		char fnw[128] = "out/", fngpx[128] = "out/";
		strcat(fnw, argv[i]);
		strcat(fngpx, argv[i]);
		strcat(fnw, ".csv");
		strcat(fngpx, ".gpx");
		bool file_created = false;
		FILE *fw;
		char pr_buf[4096];

		while(1)
		{
			size_t n = fread(&smpl, 1, sizeof(smpl), fp);
			if(n == 0)
			{
				if(ferror(fp))
				{
					perror("Error reading file");
					fclose(fp);
					return EXIT_FAILURE;
				}
				break; // EOF
			}
			if(smpl.pvt.fixType && (smpl.pvt.flags & 1))
			{
				if(!file_created)
				{
					fw = fopen(fnw, "wb");
					if(!fw)
					{
						printf("file write create error: %s\n", fnw);
						perror("Error opening write file");
						return EXIT_FAILURE;
					}
					// fprintf(fw, "ts,a0,a1,a2,g0,g1,g2,itow,year,month,day,hour,min,sec,valid,tAcc,nano,fixType,flags,flags2,numSV,lon,lat,height,hMSL,hAcc,vAcc,velN,velE,velD,gSpeed,headMot,sAcc,headAcc,pDOP\n");
					fprintf(fw, "ts,a0,a1,a2,g0,g1,g2,hour,min,sec,valid,nano,"
								"fixType,flags,flags2,numSV,lon,lat,"
								"height,hMSL,velN,velE,velD,gSpeed,headMot,"
								"hAcc,vAcc,headAcc,sAcc,pDOP\n");
					gpx_start(fngpx, &smpl.pvt);

					file_created = true;
				}
				gpx_data(&smpl.pvt);
				int i = 0;
				fprintf(fw, "%d,", smpl.t);
				fprintf(fw, "%d,", smpl.accel_raw[0]);
				fprintf(fw, "%d,", smpl.accel_raw[1]);
				fprintf(fw, "%d,", smpl.accel_raw[2]);
				fprintf(fw, "%d,", smpl.gyro_raw[0]);
				fprintf(fw, "%d,", smpl.gyro_raw[1]);
				fprintf(fw, "%d,", smpl.gyro_raw[2]);
				// fprintf(fw, "%d,", smpl.pvt.iTOW);

				// fprintf(fw, "%d,", smpl.pvt.year);
				// fprintf(fw, "%d,", smpl.pvt.month);
				// fprintf(fw, "%d,", smpl.pvt.day);
				fprintf(fw, "%d,", smpl.pvt.hour);
				fprintf(fw, "%d,", smpl.pvt.min);
				fprintf(fw, "%d,", smpl.pvt.sec);
				fprintf(fw, "%d,", smpl.pvt.valid);
				// fprintf(fw, "%d,", smpl.pvt.tAcc);
				fprintf(fw, "%d,", smpl.pvt.nano);

				fprintf(fw, "%d,", smpl.pvt.fixType);
				fprintf(fw, "%d,", smpl.pvt.flags);
				fprintf(fw, "%d,", smpl.pvt.flags2);
				fprintf(fw, "%d,", smpl.pvt.numSV);
				fprintf(fw, "%.7f,", (double)smpl.pvt.lon * 1e-7);
				fprintf(fw, "%.7f,", (double)smpl.pvt.lat * 1e-7);

				fprintf(fw, "%.3f,", (double)smpl.pvt.height * 1e-3);
				fprintf(fw, "%.3f,", (double)smpl.pvt.hMSL * 1e-3);
				fprintf(fw, "%.4f,", (double)smpl.pvt.velN * 0.0036);
				fprintf(fw, "%.4f,", (double)smpl.pvt.velE * 0.0036);
				fprintf(fw, "%.4f,", (double)smpl.pvt.velD * 0.0036);
				fprintf(fw, "%.4f,", (double)smpl.pvt.gSpeed * 0.0036);
				fprintf(fw, "%.5f,", (double)smpl.pvt.headMot * 1e-5);

				fprintf(fw, "%.3f,", (double)smpl.pvt.hAcc * 1e-3);
				fprintf(fw, "%.3f,", (double)smpl.pvt.vAcc * 1e-3);
				fprintf(fw, "%.5f,", (double)smpl.pvt.headAcc * 1e-5);
				fprintf(fw, "%.4f,", (double)smpl.pvt.sAcc * 0.0036);
				fprintf(fw, "%.2f,", (double)smpl.pvt.pDOP * 0.01);

				fprintf(fw, "\n");
			}
		}
		if(file_created)
		{
			gpx_end();
			fclose(fw);
		}
		fclose(fp);
	}
	return EXIT_SUCCESS;
}


#ifndef _TIS_SENSOR_PARTS_H_
#define _TIS_SENSOR_PARTS_H_

enum TIS_SENSOR_MODEL
{
	TIS_SENSOR_MODEL_UNKNOWN,

	TIS_SENSOR_MODEL_IMX290,	// MH[12]
	TIS_SENSOR_MODEL_IMX390,	// MH3
	TIS_SENSOR_MODEL_IMX335,	// MH[45]
	TIS_SENSOR_MODEL_IMX296,	// MH[9A]
	TIS_SENSOR_MODEL_IMX297,	// MH[BC]
	TIS_SENSOR_MODEL_AR0234,	// MH[DE]

	TIS_SENSOR_MODEL_IMX415,	// MH[FG] ??
};

enum TIS_SENSOR_TYPE
{
	TIS_SENSOR_TYPE_UNKNOWN,

	TIS_SENSOR_TYPE_COLOR,
	TIS_SENSOR_TYPE_MONO
};

enum TIS_ADAPTER_MODEL
{
	TIS_ADAPTER_MODEL_UNKNOWN,

	TIS_ADAPTER_MODEL_SER953_FAKRA,
	TIS_ADAPTER_MODEL_SER953_IP67,
	TIS_ADAPTER_MODEL_NANO_MIPI,
};

#define TIS_SENSOR_PART_DESCRIPTION_LENGTH	64

struct tis_sensor_board_part
{
	enum TIS_SENSOR_MODEL	sensor_model;
	enum TIS_SENSOR_TYPE	sensor_type;

	int						part_revision;

	char					description[TIS_SENSOR_PART_DESCRIPTION_LENGTH];
};

struct tis_adapter_board_part_caps
{
	bool has_picoblade_connector;
	bool has_fpdlink;
};

struct tis_adapter_board_part
{
	enum TIS_ADAPTER_MODEL						adapter_model;

	int											part_revision;

	char										description[TIS_SENSOR_PART_DESCRIPTION_LENGTH];

	const struct tis_adapter_board_part_caps*	caps;
};

void tis_sensor_init_sensor_board_part( struct tis_sensor_board_part* sensor_board_part );
void tis_sensor_init_adapter_board_part( struct tis_adapter_board_part* adapter_board_part );

int tis_sensor_parse_sensor_board_part( const char* part_barcode, struct tis_sensor_board_part* sensor_board_part );
int tis_sensor_parse_adapter_board_part( const char* part_barcode, struct tis_adapter_board_part* adapter_board_part );

#endif // _TIS_SENSOR_PARTS_H_

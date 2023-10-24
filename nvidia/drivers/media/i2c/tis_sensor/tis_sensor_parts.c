
#include <linux/module.h>

#include "tis_sensor_parts.h"


struct tis_adapter_board_info
{
	enum TIS_ADAPTER_MODEL				adapter_model;
	int									min_revision;
	int									max_revision;
	struct tis_adapter_board_part_caps	caps;
};

static const struct tis_adapter_board_info s_known_adapter_boards[] =
{
	{
		TIS_ADAPTER_MODEL_SER953_FAKRA,
		0, 0,
		{
			.has_picoblade_connector = true,
			.has_fpdlink = true,
		}
	},
	{
		TIS_ADAPTER_MODEL_SER953_IP67,
		0, 0,
		{
			.has_picoblade_connector = false,
			.has_fpdlink = true,
		}
	},
	{
		TIS_ADAPTER_MODEL_NANO_MIPI,
		0, 0,
		{
			.has_picoblade_connector = true
		}
	},
};

static const struct tis_adapter_board_part_caps default_adapter_no_caps = {};

void tis_sensor_init_sensor_board_part( struct tis_sensor_board_part* sensor_board_part )
{
}
EXPORT_SYMBOL(tis_sensor_init_sensor_board_part);

void tis_sensor_init_adapter_board_part( struct tis_adapter_board_part* adapter_board_part )
{
	adapter_board_part->caps = &default_adapter_no_caps;
}
EXPORT_SYMBOL(tis_sensor_init_adapter_board_part);


static int tis_sensor_parse_part( const char* part_barcode, char part_id[6], int *part_revision, char description[TIS_SENSOR_PART_DESCRIPTION_LENGTH] )
{
	// Example code
	// MF-y-MH1-xxxxxxxx-01.00-MIPI-HB FH28 color IMX290
	// Output part_id: MFMH1, this is the part-unique part of the code
	// Output part_revision: 100

	int revision_major, revision_minor;
	int ret;

	char revision_major_str[3] = {};
	char revision_minor_str[3] = {};
	strncpy( revision_major_str, part_barcode + 18, 2 );
	strncpy( revision_minor_str, part_barcode + 21, 2 );

	ret = kstrtoint( revision_major_str, 10, &revision_major );
	if( ret < 0 )
	{
		pr_err("%s: kstrtoint(%s) failed (%d)", __func__, revision_major_str, ret);
		return ret;
	}

	ret = kstrtoint( revision_minor_str, 10, &revision_minor );
	if( ret < 0 )
	{
		pr_err("%s: kstrtoint(%s) failed (%d)", __func__, revision_minor_str, ret);
		return ret;
	}

	*part_revision = revision_major * 100 + revision_minor;

	strncpy( part_id, part_barcode, 2 );
	strncpy( part_id + 2, part_barcode + 5, 3 );
	part_id[5] = '\0';

	snprintf(description, TIS_SENSOR_PART_DESCRIPTION_LENGTH, "%.*s %d.%02d", TIS_SENSOR_PART_DESCRIPTION_LENGTH - 10, part_barcode + 24, revision_major, revision_minor );

	return 0;	
}

int tis_sensor_parse_sensor_board_part( const char* part_barcode, struct tis_sensor_board_part* sensor_board_part )
{
	// Example codes:	
	// MF-y-MH1-xxxxxxxx-01.00-MIPI-HB FH28 color IMX290
	// MF-y-MH2-xxxxxxxx-01.00-MIPI-HB FH28 mono IMX290
	// MF-y-MH3-xxxxxxxx-01.00-MIPI-HB FH28 color IMX390
	// MF-y-MH4-xxxxxxxx-01.00-MIPI-HB FH28 color IMX335
	// MF-y-MH5-xxxxxxxx-01.00-MIPI-HB FH28 mono IMX335
	// MF-y-MH9-xxxxxxxx-01.00-MIPI-HB FH28 color IMX296
	// MF-y-MHA-xxxxxxxx-01.00-MIPI-HB FH28 mono IMX296
	// MF-y-MHB-xxxxxxxx-01.00-MIPI-HB FH28 color IMX297
	// MF-y-MHC-xxxxxxxx-01.00-MIPI-HB FH28 mono IMX297
	// MF-y-MHD-xxxxxxxx-01.00-MIPI-HB FH28 color AR0234
	// MF-y-MHE-xxxxxxxx-01.00-MIPI-HB FH28 mono AR0234
	// MF-y-MHF color IMX415
	// MF-y-MHG mono IMX415

	char part_id[6] = {};
	int ret;

	sensor_board_part->part_revision = 0;
	sensor_board_part->sensor_model = TIS_SENSOR_MODEL_UNKNOWN;
	sensor_board_part->sensor_type = TIS_SENSOR_TYPE_UNKNOWN;
	
	ret = tis_sensor_parse_part( part_barcode, part_id, &sensor_board_part->part_revision, sensor_board_part->description );
	if( ret < 0 )
		return ret;
	
	if( strcmp(part_id, "MFMH1") == 0 )
	{
		sensor_board_part->sensor_model = TIS_SENSOR_MODEL_IMX290;
		sensor_board_part->sensor_type = TIS_SENSOR_TYPE_COLOR;
	}
	else if( strcmp(part_id, "MFMH2") == 0 )
	{
		sensor_board_part->sensor_model = TIS_SENSOR_MODEL_IMX290;
		sensor_board_part->sensor_type = TIS_SENSOR_TYPE_MONO;
	}
	else if( strcmp(part_id, "MFMH3") == 0 )
	{
		sensor_board_part->sensor_model = TIS_SENSOR_MODEL_IMX390;
		sensor_board_part->sensor_type = TIS_SENSOR_TYPE_COLOR;
	}
	else if( strcmp(part_id, "MFMH4") == 0 )
	{
		sensor_board_part->sensor_model = TIS_SENSOR_MODEL_IMX335;
		sensor_board_part->sensor_type = TIS_SENSOR_TYPE_COLOR;
	}
	else if( strcmp(part_id, "MFMH5") == 0 )
	{
		sensor_board_part->sensor_model = TIS_SENSOR_MODEL_IMX335;
		sensor_board_part->sensor_type = TIS_SENSOR_TYPE_MONO;
	}
	else if( strcmp(part_id, "MFMH9") == 0 )
	{
		sensor_board_part->sensor_model = TIS_SENSOR_MODEL_IMX296;
		sensor_board_part->sensor_type = TIS_SENSOR_TYPE_COLOR;
	}
	else if( strcmp(part_id, "MFMHA") == 0 )
	{
		sensor_board_part->sensor_model = TIS_SENSOR_MODEL_IMX296;
		sensor_board_part->sensor_type = TIS_SENSOR_TYPE_MONO;
	}
	else if( strcmp(part_id, "MFMHB") == 0 )
	{
		sensor_board_part->sensor_model = TIS_SENSOR_MODEL_IMX297;
		sensor_board_part->sensor_type = TIS_SENSOR_TYPE_COLOR;
	}
	else if( strcmp(part_id, "MFMHC") == 0 )
	{
		sensor_board_part->sensor_model = TIS_SENSOR_MODEL_IMX297;
		sensor_board_part->sensor_type = TIS_SENSOR_TYPE_MONO;
	}
	else if( strcmp(part_id, "MFMHD") == 0 )
	{
		sensor_board_part->sensor_model = TIS_SENSOR_MODEL_AR0234;
		sensor_board_part->sensor_type = TIS_SENSOR_TYPE_COLOR;
	}
	else if( strcmp(part_id, "MFMHE") == 0 )
	{
		sensor_board_part->sensor_model = TIS_SENSOR_MODEL_AR0234;
		sensor_board_part->sensor_type = TIS_SENSOR_TYPE_MONO;
	}
	else if( strcmp(part_id, "MFMHF") == 0 )
	{
		sensor_board_part->sensor_model = TIS_SENSOR_MODEL_IMX415;
		sensor_board_part->sensor_type = TIS_SENSOR_TYPE_COLOR;
	}
	else if( strcmp(part_id, "MFMHG") == 0 )
	{
		sensor_board_part->sensor_model = TIS_SENSOR_MODEL_IMX415;
		sensor_board_part->sensor_type = TIS_SENSOR_TYPE_MONO;
	}
	else
	{
		pr_err("%s: Unexpected part_id '%s'", __func__, part_id);
		return -EINVAL;
	}

	return ret;
}
EXPORT_SYMBOL(tis_sensor_parse_sensor_board_part);

static int lookup_adapter_board_caps( struct tis_adapter_board_part* adapter_board_part )
{
	int i;

	for( i = 0; i < ARRAY_SIZE(s_known_adapter_boards); ++i )
	{
		const struct tis_adapter_board_info* info = &s_known_adapter_boards[i];

		if( info->adapter_model != adapter_board_part->adapter_model )
			continue;
		if( info->min_revision && info->min_revision > adapter_board_part->part_revision )
			continue;
		if( info->max_revision && info->max_revision < adapter_board_part->part_revision )
			continue;

		adapter_board_part->caps = &info->caps;
		return 0;
	}

	pr_err("%s: No adapter info for board part '%s'. No capabilities have been set. A driver upgrade may be required.", __func__, adapter_board_part->description);

	adapter_board_part->caps = &default_adapter_no_caps;

	return 0;
}

int tis_sensor_parse_adapter_board_part( const char* part_barcode, struct tis_adapter_board_part* adapter_board_part )
{
	// Example codes:
	// MF-y-NMA-xxxxxxxx-01.20-Jetson Nano MIPI Adapter
	// MF-y-LS1-xxxxxxxx-01.00-FPD-Link Ser953/FH28-Fakra
	// MF-y-LS1-xxxxxxxx-01.20-FPD-Link Ser953/FH28-Fakra
	// MF-y-LS2-xxxxxxxx-01.20-FPD-Link Ser953/FH28-IP67

	char part_id[6] = {};
	int ret;

	adapter_board_part->part_revision = 0;
	adapter_board_part->adapter_model = TIS_ADAPTER_MODEL_UNKNOWN;
	adapter_board_part->caps = &default_adapter_no_caps;
	
	ret = tis_sensor_parse_part( part_barcode, part_id, &adapter_board_part->part_revision, adapter_board_part->description );
	if( ret < 0 )
		return ret;

	if( strcmp(part_id, "MFNMA") == 0 )
	{
		adapter_board_part->adapter_model = TIS_ADAPTER_MODEL_NANO_MIPI;
	}
	else if( strcmp(part_id, "MFLS1") == 0 )
	{
		adapter_board_part->adapter_model = TIS_ADAPTER_MODEL_SER953_FAKRA;
	}
	else if( strcmp(part_id, "MFLS2") == 0 )
	{
		adapter_board_part->adapter_model = TIS_ADAPTER_MODEL_SER953_IP67;
	}
	else
	{
		pr_err("%s: Unexpected part_id '%s'", __func__, part_id);
		return -EINVAL;
	}

	return lookup_adapter_board_caps( adapter_board_part );
}
EXPORT_SYMBOL(tis_sensor_parse_adapter_board_part);

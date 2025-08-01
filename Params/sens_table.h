/*
 * sens_table.h
 *
 *  Created on: 2023/06/24
 *      Author: sato1
 */

#ifndef INCLUDE_SENS_TABLE_H_
#define INCLUDE_SENS_TABLE_H_

const static int16_t sens_side_length_table[]=
{
		25,
		30,
		35,
		40,
		45,
		50,
		55,
		60,
		65,
		70,
		75,
		80
};

const static int16_t sens_sr_table[]=
{
		1970,//25
		1240,//30
		860,//35
		660,//40
		540,//45
		420,//50
		360,//55
		300,//60
		270,//65
		240,//70
		200,//75
		180//80
};

const static int16_t sens_sl_table[]=
{
		1400,//25
		950,//30
		710,//35
		570,//40
		460,//45
		380,//50
		310,//55
		270,//60
		240,//65
		200,//70
		185,//75
		170//80
};
const static int16_t sens_front_length_table[]=
{
		40,
		45,
		50,
		55,
		60,
		65,
		70,
		75,
		80,
		85,
		90,
		95,
		100,
		105,
		110,
		115,
		120,
		125
};

const static int16_t sens_fr_table[]=
{
		3470,//40
		2600,//45
		1900,//50
		1520,//55
		1240,//60
		1010,//65
		850,//70
		740,//75
		630,//80
		570,//85
		510,//90
		460,//95
		420,//100
		380,//105
		350,//110
		320,//115
		310,//120
		300
};

const static int16_t sens_fl_table[]=
{
		3600,//40
		2750,//45
		2050,//50
		1580,//55
		1350,//60
		1090,//65
		950,//70
		810,//75
		710,//80
		650,//85
		570,//90
		500,//95
		450,//100
		420,//105
		400,//110
		350,//115
		330,//120
		300//125
};



#endif /* INCLUDE_SENS_TABLE_H_ */

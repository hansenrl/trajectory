/*
 * ConvertData.cpp
 *
 *  Created on: Oct 31, 2014
 *      Author: ross
 */

#include "csv_parser.hpp"
#include <string>
#include <iostream>
#include <vector>
#include <set>

int main(void)
{
	const string filename("/home/ross/TrackTruth/TRAIN/20091021_truth_rset0_frames0100-0611.csv");
	//const string filename("/home/ross/csv-parser-cplusplus-read-only/examples/example_input.csv");
	const char field_terminator = ',';
	const char line_terminator = '\n';
	//const char enclosure_char = '"';
	//const char * filename = "example_input.csv";

	csv_parser file_parser;

	file_parser.set_skip_lines(1);
	file_parser.init(filename.c_str());
	//file_parser.set_enclosed_char(enclosure_char, ENCLOSURE_OPTIONAL);
	file_parser.set_field_term_char(field_terminator);
	file_parser.set_line_term_char(line_terminator);

	unsigned int row_count = 1U;

	struct point {
		int id;
		double lat;
		double lon;
		int frame;
	};

	double min_lat = 90;
	double min_lon = 180;

	while(file_parser.has_more_rows()){
		/*unsigned int i = 0;

		csv_row row = file_parser.get_row();

        for (i = 0; i < row.size(); i++)
        {
                printf("COLUMN %02d : %s\n", i + 1U, row[i].c_str());
        }

        printf("====================================================================\n");
        printf("END OF ROW %02d\n", row_count);
        printf("====================================================================\n");*/

		csv_row row = file_parser.get_row();

		cout << row[0] << " " << row[1] << " " << row[2] << " " << row[6];
		int id = atoi(row[0].c_str());
		double lat = atof(row[1].c_str());
		double lon = atof(row[2].c_str());
		int frame = atoi(row[6].c_str());

		if( lat < min_lat ) { min_lat = lat; }
		if( lon < min_lon ) { min_lon = lon; }



		cout << "\n";

        row_count++;
	}

	return 0;
}




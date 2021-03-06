/*
 * ConvertData.cpp
 *
 * Converts data in the format of the WPAFB 2009 dataset to the .traj format expected by TrajData
 * See https://www.sdms.afrl.af.mil/index.php?collection=wpafb2009 for details about the WPAFB 2009 dataset
 *
 *  Created on: Oct 31, 2014
 *      Author: ross
 */

#include "csv_parser.hpp"
#include <string>
#include <iostream>
#include <iomanip>
#include <limits>
#include <fstream>
#include <vector>
#include <map>

int main(void)
{
	const string filename("/home/ross/TrackTruth/TRAIN/20091021_truth_rset0_frames0100-0611.csv");
	//const string filename("/home/ross/csv-parser-cplusplus-read-only/examples/example_input.csv");
	const char field_terminator = ',';
	const char line_terminator = '\n';
	const double m_per_lon = 85417;
	const double m_per_lat = 111030;

	csv_parser file_parser;

	file_parser.set_skip_lines(1);
	file_parser.init(filename.c_str());
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

	map<int, vector<point> > points;

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

		//if(points.size() > 10000)
		//	break;

		csv_row row = file_parser.get_row();

		//cout << row[0] << " " << row[1] << " " << row[2] << " " << row[6];
		int id = atoi(row[0].c_str());
		//double lat = atof(row[1].c_str());
		double lat = atof(row[1].c_str());
		//printf("%lf ", lat);
		//cout << setprecision(numeric_limits<double>::digits10) << lat << " " << row[1].c_str() << "\n";
		double lon = atof(row[2].c_str());
		int frame = atoi(row[6].c_str());

		if(frame > 400)
			continue;

		if( lat < min_lat ) { min_lat = lat; }
		if( lon < min_lon ) { min_lon = lon; }

		point new_point;
		new_point.id = id; new_point.lat = lat; new_point.lon = lon; new_point.frame = frame;

		points[id].push_back(new_point);


		//cout << "\n";
		if(row_count % 10000 == 0){
			cout << "Row " << row_count << "\n";
		}

        row_count++;
	}

	// remove short paths
	for( auto iter = points.begin(); iter != points.end(); iter++){
		if(iter->second.size() < 20){
			points.erase(iter++);
		}
	}

	ofstream ofile;
	ofile.open("/home/ross/afrl_data.tra");
	ofile << "2\n";
	ofile << points.size() << "\n";

	for( auto map_pair : points){
		ofile << map_pair.first - 1 << " " << map_pair.second.size();
		for( auto p : map_pair.second){
			ofile << " " << (p.lon - min_lon) * m_per_lon << " " << (p.lat - min_lat) * m_per_lat;
		}
		ofile << "\n";
	}
	ofile.close();

	return 0;
}




#include <iostream>
#include <stdio.h>
#include <math.h> 
#include <algorithm>
#include <iostream>
#include <string>
#include <fstream> 
#include <vector> 
#include <pcl/point_types.h>
#include <pcl/point_cloud.h> 
#include "EMst.h"
#include <filesystem>

//函数声明
int main(int argc, char* argv[])
{
	EMst g("G://Code//data//1_AIM//AIM.pcd", false);
	int K = 30;
	pcl::PointCloud<RawPointType>::Ptr m_knnpoint(new pcl::PointCloud<RawPointType>);
	int T = 200;//粒子扩增T次
	int Pa = 700;//粒子的个数
	vector<trace> paths;
	m_knnpoint = (g.raw).makeShared();
	g.get_new_widght(m_knnpoint, Pa, T, K, paths);
	g.outputPLFile(m_knnpoint, paths, "//./result/1_AIM/1.6-700-500-RD.pl");
}

#include "EMst.h"
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <numeric>
#include <cstdlib>
#include <ctime>
#include <random>
#include <filesystem>

vector<int> RandomGenerateParticle(const int range, const int Pa)
{
	//1. 随机生成Pnum个粒子,存到容器中 创建包含所有可能整数的向量
	std::vector<int> allIntegers;
	if (range <= 0) {
		// 处理 range 为非正数的情况
		cout << "range 为非正数" << endl;
		return allIntegers; // 返回一个空的向量
	}
	for (int i = 0; i < range; ++i) {
		allIntegers.push_back(i);
	}
	// 检查 allIntegers 是否为空
	if (allIntegers.empty()) {
		// 处理 allIntegers 为空的情况
		cout << "allIntegers 为空" << endl;
		return allIntegers; // 返回一个空的向量
	}
	// 使用随机设备生成器
	std::random_device rd;
	std::mt19937 g(rd());
	// 打乱序列
	std::shuffle(allIntegers.begin(), allIntegers.end(), g);
	// 检查 allIntegers 的大小是否足够
	if (Pa > allIntegers.size()) {
		// 处理 Pa 大于 allIntegers 大小的情况
		return allIntegers; // 返回全部元素
	}
	// 从打乱后的序列中选择前 numElements 个元素
	std::vector<int> randomIntegers(allIntegers.begin(), allIntegers.begin() + Pa);
	// 输出结果
	std::cout << "随机生成的" << Pa << "个不相同的整数：" << std::endl;
	return randomIntegers;
}
//****************************|||||||||||||粒子滤波的核心函数||||||||||||||*********************************
void EMst::get_new_widght(p_rawPoint P, int Pa, int T, int k, vector<trace>& paths)
{
	//1--定义粒子存储结构
	int Psize = P->points.size();
	cout << "P->points.size:  " << Psize << endl;
	vector<int> randomStart = RandomGenerateParticle(P->points.size(), Pa);//从所有点中随机选Pa个
	paths.resize(randomStart.size());//vector 是一种动态数组
	for (int i = 0; i < randomStart.size(); i++) {
		paths[i].index.push_back(randomStart[i]);
		paths[i].weight = 1.0 / randomStart.size();
		//cout << "定义粒子存储结构paths[i].weight: " << paths[i].weight << endl;
		//paths[i].weight = 1.0f / randomStart.size();
	}
	//输出随机选择的Pa个粒子
	outputPLFile(P, paths, "//./result/1_AIM/1.6-RD.pl");
	releaseVector(randomStart);
	//2--计算领域内分布律，先对KNN邻域进行构建
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::copyPointCloud(*P, *cloud);
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(cloud);
	vector<int> kneighbor;
	vector<float> dist;
	//3--粒子扩增
	for (int iter = 0; iter < T; iter++)
	{
		cout << iter << endl;
		for (int j = 0; j < paths.size(); j++)
		{
			//cout << "开始扩增" << endl;
			int curindex = *paths[j].index.rbegin();
			pcl::PointXYZ curPt;
			curPt.x = P->points[curindex].x;
			curPt.y = P->points[curindex].y;
			curPt.z = P->points[curindex].z;
			//cout << "建立Kdtree" << endl;
			kdtree.nearestKSearch(curPt, k, kneighbor, dist); //计算邻域内分布律-------const int neiborknei = 30;
			int samplingIndex = 0;
			for (int t = 0; t < 5; ) {//5是一个自定义的范围阈值，在最近的五个点中找到不包括在之前的路径中的扩增点
				//cout << "采样：" << samplingIndex << endl;
				auto it = find(paths[j].index.begin(), paths[j].index.end(), kneighbor[samplingIndex + 1]);//find函数是标准库 <algorithm> 中的函数，用于在指定范围内查找特定值
				if (it != paths[j].index.end()) {
					samplingIndex++;
					t++;
				}
				else break;
			}
			paths[j].index.push_back(kneighbor[samplingIndex + 1]);
			paths[j].weight += 1.0 / dist[samplingIndex + 1];
			releaseVector(kneighbor);
			releaseVector(dist);
		}
		cout << "扩增完成，计算权" << endl;
		vector<float> patheweight;
		for (int j = 0; j < paths.size(); j++) {
			patheweight.push_back(paths[j].weight);
		}
		normalizeVector(patheweight);
		float weightsqr = 0.0;
		for (int j = 0; j < paths.size(); j++) {
			paths[j].weight = patheweight[j];
			weightsqr += pow(paths[j].weight, 2.0);
		}
		//cout << "计算有效样本：" << endl;
		int Neffective = ceil(1.0f / weightsqr);
		cout << "计算有效样本：" << Neffective << endl;
		cout << "权计算完成" << endl;
	}
}
//输出pl文件的函数（√）
void EMst::outputPLFile(p_rawPoint P, const vector<trace>& paths, string name)
{
	string filename = path + name; // 使用预定义的路径信息拼接完整的文件路径
	ofstream PLfile1(filename, ios::app); // 打开文件流
	cout << "共有路径Path.size =" << paths.size() << endl;
	for (int m = 0; m < paths.size(); m++)
	{
		PLfile1 << "GOCAD PLine \n";
		PLfile1 << "HEADER{\n";
		PLfile1 << "name:" << "1-random-ponints" << m << "\n";
		PLfile1 << "value:" << paths[m].weight << "\n";
		PLfile1 << "}\n";
		PLfile1 << "ILINE\n";
		for (int i = 0; i < paths[m].index.size(); i++)
		{
			int num = paths[m].index[i];
			PLfile1 << "VRTX " << i << " " << P->points[num].x << " " << P->points[num].y << " " << P->points[num].z << "\n";
		}
		PLfile1 << "END";
	}
	PLfile1.close(); // 关闭文件流
}

template <typename T> void releaseVector(vector<T>& vec) {
	vec.clear();
	vector<T>(vec).swap(vec);
}

template <typename T> void normalizeVector(std::vector<T>& vec) {
	// 计算向量中所有元素的和
	float sum = 0.0f;
	for (float element : vec) {
		sum += element;
	}

	// 归一化向量
	for (float& element : vec) {
		element /= sum;
	}
}

EMst::EMst(string allPointCloudPath, bool flag)
{
	if (flag) {
		pcl::io::loadPCDFile(allPointCloudPath, this->mypoint);
		cout << "点云数据输入完成." << endl;
	}
	else {
		pcl::io::loadPCDFile(allPointCloudPath, this->raw);
		cout<<"点云数据输入完成."  <<endl;
		int Point = this->raw.size();
		cout << "全部点的数量" << Point << "：" << endl;
		for (int i = 0; i < Point; i++) {
			float p0 = 0.5f *(this->raw.points[i].intensity + this->raw.points[i].gvalue);
			float p1 = 1 - p0;
			if (p0 * p1 < 0)cout << "错误发生在第"<<i<< "行" << endl;
			else {
				RawPointType p;
				p.x = this->raw.points[i].x;
				p.y = this->raw.points[i].y;
				p.z = this->raw.points[i].z;
				p.intensity = this->raw.points[i].intensity;
				p.gvalue = this->raw.points[i].gvalue;
				p.Tlambda0 = this->raw.points[i].Tlambda0;
				p.Tlambda1 = this->raw.points[i].Tlambda1;
				p.Tlambda2 = this->raw.points[i].Tlambda2;
			}
		}
	}
	//全部点云数据构图
	int Point = this->raw.size();
	//cout << "点云个数Point" << Point <<": " << endl;
	//define the points in KNN graph
	for (int i = 0; i < Point; i++) {
		float p0 = 0.5f *(this->raw.points[i].intensity + this->raw.points[i].gvalue);
		//ProLine = p0;
		//if (ProLine > threshold_p) 
		this->rawknnPoint.push_back(this->raw.points[i]);
	}
	int rawknnPointSIZE = this->rawknnPoint.size();
	this->Graph.adjList = new AdjList[rawknnPointSIZE];
	this->Graph.numVertsxes = rawknnPointSIZE;
	//cout << "点云个数rawknnPoint" << ": " << rawknnPointSIZE  << endl;//点云个数Point81366 int NumknnPoint = this->rawknnPoint.size();
	GraphAdjList *G = &this->Graph;
	G->adjList = new AdjList[rawknnPointSIZE];
	G->numVertsxes = rawknnPointSIZE;
	for (int i = 0; i < rawknnPointSIZE; i++) {
		G->adjList[i].data = i;
		G->adjList[i].degree = 0;
		G->adjList[i].firstedge = NULL;
	}
}
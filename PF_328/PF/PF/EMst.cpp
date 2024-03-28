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
	//1. �������Pnum������,�浽������ �����������п�������������
	std::vector<int> allIntegers;
	if (range <= 0) {
		// ���� range Ϊ�����������
		cout << "range Ϊ������" << endl;
		return allIntegers; // ����һ���յ�����
	}
	for (int i = 0; i < range; ++i) {
		allIntegers.push_back(i);
	}
	// ��� allIntegers �Ƿ�Ϊ��
	if (allIntegers.empty()) {
		// ���� allIntegers Ϊ�յ����
		cout << "allIntegers Ϊ��" << endl;
		return allIntegers; // ����һ���յ�����
	}
	// ʹ������豸������
	std::random_device rd;
	std::mt19937 g(rd());
	// ��������
	std::shuffle(allIntegers.begin(), allIntegers.end(), g);
	// ��� allIntegers �Ĵ�С�Ƿ��㹻
	if (Pa > allIntegers.size()) {
		// ���� Pa ���� allIntegers ��С�����
		return allIntegers; // ����ȫ��Ԫ��
	}
	// �Ӵ��Һ��������ѡ��ǰ numElements ��Ԫ��
	std::vector<int> randomIntegers(allIntegers.begin(), allIntegers.begin() + Pa);
	// ������
	std::cout << "������ɵ�" << Pa << "������ͬ��������" << std::endl;
	return randomIntegers;
}
//****************************|||||||||||||�����˲��ĺ��ĺ���||||||||||||||*********************************
void EMst::get_new_widght(p_rawPoint P, int Pa, int T, int k, vector<trace>& paths)
{
	//1--�������Ӵ洢�ṹ
	int Psize = P->points.size();
	cout << "P->points.size:  " << Psize << endl;
	vector<int> randomStart = RandomGenerateParticle(P->points.size(), Pa);//�����е������ѡPa��
	paths.resize(randomStart.size());//vector ��һ�ֶ�̬����
	for (int i = 0; i < randomStart.size(); i++) {
		paths[i].index.push_back(randomStart[i]);
		paths[i].weight = 1.0 / randomStart.size();
		//cout << "�������Ӵ洢�ṹpaths[i].weight: " << paths[i].weight << endl;
		//paths[i].weight = 1.0f / randomStart.size();
	}
	//������ѡ���Pa������
	outputPLFile(P, paths, "//./result/1_AIM/1.6-RD.pl");
	releaseVector(randomStart);
	//2--���������ڷֲ��ɣ��ȶ�KNN������й���
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::copyPointCloud(*P, *cloud);
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(cloud);
	vector<int> kneighbor;
	vector<float> dist;
	//3--��������
	for (int iter = 0; iter < T; iter++)
	{
		cout << iter << endl;
		for (int j = 0; j < paths.size(); j++)
		{
			//cout << "��ʼ����" << endl;
			int curindex = *paths[j].index.rbegin();
			pcl::PointXYZ curPt;
			curPt.x = P->points[curindex].x;
			curPt.y = P->points[curindex].y;
			curPt.z = P->points[curindex].z;
			//cout << "����Kdtree" << endl;
			kdtree.nearestKSearch(curPt, k, kneighbor, dist); //���������ڷֲ���-------const int neiborknei = 30;
			int samplingIndex = 0;
			for (int t = 0; t < 5; ) {//5��һ���Զ���ķ�Χ��ֵ�����������������ҵ���������֮ǰ��·���е�������
				//cout << "������" << samplingIndex << endl;
				auto it = find(paths[j].index.begin(), paths[j].index.end(), kneighbor[samplingIndex + 1]);//find�����Ǳ�׼�� <algorithm> �еĺ�����������ָ����Χ�ڲ����ض�ֵ
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
		cout << "������ɣ�����Ȩ" << endl;
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
		//cout << "������Ч������" << endl;
		int Neffective = ceil(1.0f / weightsqr);
		cout << "������Ч������" << Neffective << endl;
		cout << "Ȩ�������" << endl;
	}
}
//���pl�ļ��ĺ������̣�
void EMst::outputPLFile(p_rawPoint P, const vector<trace>& paths, string name)
{
	string filename = path + name; // ʹ��Ԥ�����·����Ϣƴ���������ļ�·��
	ofstream PLfile1(filename, ios::app); // ���ļ���
	cout << "����·��Path.size =" << paths.size() << endl;
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
	PLfile1.close(); // �ر��ļ���
}

template <typename T> void releaseVector(vector<T>& vec) {
	vec.clear();
	vector<T>(vec).swap(vec);
}

template <typename T> void normalizeVector(std::vector<T>& vec) {
	// ��������������Ԫ�صĺ�
	float sum = 0.0f;
	for (float element : vec) {
		sum += element;
	}

	// ��һ������
	for (float& element : vec) {
		element /= sum;
	}
}

EMst::EMst(string allPointCloudPath, bool flag)
{
	if (flag) {
		pcl::io::loadPCDFile(allPointCloudPath, this->mypoint);
		cout << "���������������." << endl;
	}
	else {
		pcl::io::loadPCDFile(allPointCloudPath, this->raw);
		cout<<"���������������."  <<endl;
		int Point = this->raw.size();
		cout << "ȫ���������" << Point << "��" << endl;
		for (int i = 0; i < Point; i++) {
			float p0 = 0.5f *(this->raw.points[i].intensity + this->raw.points[i].gvalue);
			float p1 = 1 - p0;
			if (p0 * p1 < 0)cout << "�������ڵ�"<<i<< "��" << endl;
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
	//ȫ���������ݹ�ͼ
	int Point = this->raw.size();
	//cout << "���Ƹ���Point" << Point <<": " << endl;
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
	//cout << "���Ƹ���rawknnPoint" << ": " << rawknnPointSIZE  << endl;//���Ƹ���Point81366 int NumknnPoint = this->rawknnPoint.size();
	GraphAdjList *G = &this->Graph;
	G->adjList = new AdjList[rawknnPointSIZE];
	G->numVertsxes = rawknnPointSIZE;
	for (int i = 0; i < rawknnPointSIZE; i++) {
		G->adjList[i].data = i;
		G->adjList[i].degree = 0;
		G->adjList[i].firstedge = NULL;
	}
}
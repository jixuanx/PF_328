#pragma once
#ifndef EMST_H
#define EMST_H

#include<iostream>
#include<string>
#include <stdio.h>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <vector>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common_headers.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <numeric>
#include <Eigen/Dense>
#include <array>
#include <cstdlib>
#include <ctime>
#include <random>
#include <filesystem>
//#include<omp.h>
#define path "G:/Code/new/k-mst/k-mst/lookprocess/"
typedef int VertexType;
typedef float EdgeType;
#define MAXVEX 5000
#define Inf 255800.000//Inf ��һ�����ڱ�ʾ�����ĳ�����ͨ���� Inf ����Ϊ 255800.000��������ڴ�����ʹ�� Inf ����ʾһ���ϴ��ֵ��
using namespace std;
//the structure of data in MST
struct trace {
	vector<int> index;
	float weight = 0.0;
};
typedef struct EdgeNode {
	int adjvex;
	EdgeType weight;
	struct EdgeNode *next;
}EdgeNode;
typedef struct VertexNode {
	VertexType data;
	int degree;
	EdgeNode *firstedge;
	bool visit;
	bool is_cor;
}AdjList;
typedef struct {
	AdjList *adjList;
	int numVertsxes, numEdges;
}GraphAdjList;
struct EndNode {
	int data;
	int degree;
	vector<int> lineorder;
};
struct MyPointType
{
	PCL_ADD_POINT4D;  //�õ�������4��Ԫ��      

	/*��������һ���Զ���*/
	float intensity;
	float gvalue;
	float p0;
	float p1;
	float ptype;
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW   //ȷ��new������������� 

}EIGEN_ALIGN16;   //ǿ��SSE ����
POINT_CLOUD_REGISTER_POINT_STRUCT(MyPointType,    //ע������ͺ�
(float, x, x)
(float, y, y)
(float, z, z)
(float, intensity, intensity)
(float, gvalue, gvalue)
(float, p0, p0)
(float, p1, p1)
(float, ptype, ptype)
)
struct RawPointType    //��������ͽṹ
{
	PCL_ADD_POINT4D;  //�õ�������4��Ԫ��      

	/*��������һ���Զ���*/
	float intensity;
	float gvalue;
	float gx;
	float gy;
	float gz;
	float Tlambda0;
	float Tlambda1;
	float Tlambda2;
	int iscorner;
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW   //ȷ��new������������� 

}EIGEN_ALIGN16;   //ǿ��SSE ����
POINT_CLOUD_REGISTER_POINT_STRUCT(RawPointType, //ע������ͺ�
(float, x, x)
(float, y, y)
(float, z, z)
(float, intensity, intensity)
(float, gvalue, gvalue)
(float, gx, gx)
(float, gy, gy)
(float, gz, gz)
(float, Tlambda0, Tlambda0)
(float, Tlambda1, Tlambda1)
(float, Tlambda2, Tlambda2)
(int, iscorner, iscorner)
)
typedef pcl::PointCloud<pcl::PointXYZ> PointXYZ;
typedef pcl::PointCloud<pcl::PointXYZ>::Ptr p_PointXYZ;
typedef pcl::PointCloud<MyPointType> Mypoint;//�ø��̵����� Mypoint ������ pcl::PointCloud<MyPointType>
typedef pcl::PointCloud<MyPointType>::Ptr p_Mypoint;
typedef pcl::PointCloud<RawPointType> rawPoint;
typedef pcl::PointCloud<RawPointType>::Ptr p_rawPoint;
//template<typename T, typename... U>
//void logger(T t, U... ts);
static EdgeNode * make_node(const int pos, const float distance)
{
	EdgeNode * new_node = (EdgeNode *)malloc(sizeof(EdgeNode));
	if (new_node == NULL)
		exit(1);

	new_node->next = NULL;
	new_node->weight = distance;
	new_node->adjvex = pos;

	return new_node;
}
struct EdgeInfo
{
	PCL_ADD_POINT4D;  //�õ�������4��Ԫ��      
	/*���������Զ���*/
	int start;
	int end;
	long double wight_value;
	EdgeInfo()
	{
		memset(this, 0, sizeof(EdgeInfo));
	}
};
template <typename T> void releaseVector(vector<T>& vec);
template <typename T> void normalizeVector(std::vector<T>& vec);

class EMst
{
public:
	GraphAdjList Graph;
	GraphAdjList CornerGraph;
	GraphAdjList BoundaryGraph;
	//Mypoint������������Ϊ5ά<I,g,p0,p1,ptype>
	Mypoint mypoint;
	Mypoint knnPoint;
	//rawPoint��������Ϊ8ά����<I,g,gx,gy,gz,lamb1,lamb2,lamb3>
	rawPoint raw;
	rawPoint corraw;
	rawPoint bouraw;
	rawPoint rawknnPoint;
	rawPoint cornerknnPoint;
	rawPoint boundaryknnPoint;
public:
	EMst(string allPointCloudPath, bool flag);//������ EMst ��Ĺ��캯�����ú��������ĸ�������
	void get_new_widght(p_rawPoint P, int Pnum, int time, int k, vector<trace>& paths);
	void outputPLFile(p_rawPoint P, const vector<trace>& paths, string name);
	~EMst()
	{}
};
#endif // EMST_H

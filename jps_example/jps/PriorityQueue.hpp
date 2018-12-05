#ifndef PRIORITY_QUEUE_H
#define PRIORITY_QUEUE_H

#include <vector>
#include <algorithm>

#include "Position.h"
#include "PathfindingNode.hpp"

#define ASSERT(x)

namespace JPS
{

// SortedPriorityQueue를 관리하기 위한 리스트
// 우선 순위 큐보다 성능상 유리
class UnsortedPriorityQueue
{
	int m_NextFreeNode;
	PathfindingNode** m_Array;
	const int MAX_NODE = 10000;
public:
	UnsortedPriorityQueue() : m_NextFreeNode(0) { m_Array = new PathfindingNode*[MAX_NODE]; }
	~UnsortedPriorityQueue() { delete[] m_Array; }

	void Clear() { m_NextFreeNode = 0; }
	bool Empty(void) { return m_NextFreeNode == 0; }
	void Push(PathfindingNode* node)
	{
		m_Array[m_NextFreeNode++] = node;
	}
	PathfindingNode* Pop(void)
	{
		unsigned int CheapestNodeCost = m_Array[0]->f;
		int CheapestNodeIndex = 0;

		for (int i = 1; i < m_NextFreeNode; ++i)
		{
			if (m_Array[i]->f < CheapestNodeCost)
			{
				CheapestNodeCost = m_Array[i]->f;
				CheapestNodeIndex = i;
			}
		}

		PathfindingNode* cheapestNode = m_Array[CheapestNodeIndex];
		m_Array[CheapestNodeIndex] = m_Array[--m_NextFreeNode];
		return cheapestNode;
	}

	void Fixup()
	{
	}
};

// SortedPriorityQueue를 관리하기 위한 리스트
// 우선 순위 큐보다 성능상 유리
class SortedPriorityQueue
{
	using NodeVector = std::vector<PathfindingNode*>;
	NodeVector m_Nodes;
public:
	SortedPriorityQueue() { m_Nodes.reserve(100); }
	void Push(PathfindingNode *node)
	{
		ASSERT(node);
		m_Nodes.push_back(node);
		std::push_heap(m_Nodes.begin(), m_Nodes.end(), [](const auto *a, const auto *b) { return a->f > b->f; });
	}
	PathfindingNode *Pop()
	{
		std::pop_heap(m_Nodes.begin(), m_Nodes.end(), [](const auto *a, const auto *b) { return a->f > b->f; });
		PathfindingNode *node = m_Nodes.back();
		m_Nodes.pop_back();
		return node;
	}
	bool Empty() const
	{
		return m_Nodes.empty();
	}
	void Clear()
	{
		m_Nodes.clear();
	}
	void Fixup()
	{
		std::make_heap(m_Nodes.begin(), m_Nodes.end(), [](const auto *a, const auto *b) { return a->f > b->f; });
	}
};

}
#endif


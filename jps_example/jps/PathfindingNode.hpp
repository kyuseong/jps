#ifndef PATHFINDING_NODE_H
#define PATHFINDING_NODE_H

namespace JPS
{
// 검색된 노드 정보
class PathfindingNode
{
	unsigned int flags;

public:
	PathfindingNode() {}
	PathfindingNode(const Position& p) : f(0), g(0), pos(p), parent(0), flags(0) {}
	unsigned int f;
	unsigned int g;
	Position pos;
	const PathfindingNode *parent;

	void SetOpen() { flags |= 1; }
	void SetClosed() { flags |= 2; }
	unsigned int IsOpen() const { return flags & 1; }
	unsigned int IsClosed() const { return flags & 2; }
	void Clear() { f = 0; g = 0, parent = 0; flags = 0; }
};

// 휴리스틱 계산하기
// 멘하탄
inline unsigned Manhattan(const PathfindingNode *a, const PathfindingNode *b)
{
	return abs(int(a->pos.x - b->pos.x)) + abs(int(a->pos.y - b->pos.y));
}

//유클리디안
inline unsigned Euclidean(const PathfindingNode *a, const PathfindingNode *b)
{
	float fx = float(int(a->pos.x - b->pos.x));
	float fy = float(int(a->pos.y - b->pos.y));
	return unsigned(int(sqrtf(fx*fx + fy * fy)));
}

}

#endif


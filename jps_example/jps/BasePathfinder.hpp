#ifndef PATHFINDER_H
#define PATHFINDER_H

namespace JPS
{
// 길찾기 결과
enum ePATH_RESULT
{
	PATH_RESULT_NO_PATH,
	PATH_RESULT_FOUND_PATH,
	PATH_RESULT_NEED_MORE_STEPS,
	PATH_RESULT_EMPTY_PATH
};

// 길찾기 
template <typename GRID, typename RULE>
class BasePathFinder
{
	const GRID& m_Grid;		// 그리드(맵 정보)
	RULE m_Rule;
	PathfindingNode* m_EndNode;		// 마지막 노드
	UnsortedPriorityQueue m_OpenList;	// 맵 리스트
	PathfindingNode* m_NodeTable;
	int m_NodeSize;
public:
	BasePathFinder(const GRID& g) : m_Grid(g), m_Rule(g), m_EndNode(nullptr)
	{
		const int MAX_NODE_TABLE = 10000;
		m_NodeTable = new PathfindingNode[MAX_NODE_TABLE];
		m_NodeSize = 0;
	}

	~BasePathFinder()
	{
		delete[] m_NodeTable;
	}
	// 길을 찾기
	bool FindPath(PathArray& path, Position start, Position end, unsigned int Step = 0);
	// 초기화 하기
	ePATH_RESULT Init(Position start, Position end);
	// 한 step 씩 길 찾기
	ePATH_RESULT NextStep();
	// 종료 하기 - 길 만들어 내기 
	bool Finish(PathArray& path, unsigned int step);
	
	size_t GetNodesExpanded() const { return m_NodeSize; }

private:
	// 노드 정보를 가지고 온다.
	PathfindingNode *GetNode(const Position& pos);
	// 후보자들은 확인해 나간다. - 주변이웃을 찾고 탐색한다.
	void IdentifySuccessors(const PathfindingNode *n);
	// 찾아 놓은 스택을 기반으로 길을 만든다.
	bool MakePath(PathArray& path, unsigned int step) const;
	// 직선 경로 찾기
	bool FindStraight(PathfindingNode *start);
};


// 노드 구하기(생성)
template <typename GRID, typename RULE>
PathfindingNode *BasePathFinder<GRID, RULE>::GetNode(const Position& pos)
{
	for (int i = 0; i < m_NodeSize; ++i)
	{
		if (m_NodeTable[i].pos == pos)
		{
			return &m_NodeTable[i];
		}
	}


	auto NewNode = &m_NodeTable[m_NodeSize++];
	NewNode->Clear();
	NewNode->pos = pos;
	return NewNode;
}

///////////////////////////////////////////////////////////////////////////////
// 다음 진행 (a*)
///////////////////////////////////////////////////////////////////////////////
template <typename GRID, typename RULE>
void BasePathFinder<GRID, RULE>::IdentifySuccessors(const PathfindingNode *n)
{
	// 주변 8반향에 대한 Pos
	Position PosBuffer[8];

	const unsigned int num = m_Rule.FillNeighbors(n, &PosBuffer[0]);
	for (int i = num - 1; i >= 0; --i)
	{
		Position MovePos = m_Rule.Jump(PosBuffer[i], n->pos, m_EndNode->pos);

		if (!MovePos.isValid())
			continue;

		// forced neighbours 을 찾았다.
		// 이제 MovePos 위치가 유효한 점프된 위치 이므로 실제 노드를 만들어야합니다.
		PathfindingNode *MoveNode = GetNode(MovePos);
		ASSERT(MoveNode && MoveNode != n);
		if (!MoveNode->IsClosed())
		{
			unsigned int ExtraG = Euclidean(MoveNode, n);
			unsigned int G = n->g + ExtraG;
			if (!MoveNode->IsOpen() || G < MoveNode->g)
			{
				MoveNode->g = G;
				MoveNode->f = MoveNode->g + Manhattan(MoveNode, m_EndNode);
				MoveNode->parent = n;
				if (!MoveNode->IsOpen())
				{
					m_OpenList.Push(MoveNode);
					MoveNode->SetOpen();
				}
				else
					m_OpenList.Fixup();
			}
		}
	}
}

///////////////////////////////////////////////////////////////////////////////
// 찾아 놓은 스택을 기반으로 길을 만든다.
// m_EndNode 기준으로 parent을 찾아 가면서 길을 만듦
///////////////////////////////////////////////////////////////////////////////
template <typename GRID, typename RULE>
bool BasePathFinder<GRID, RULE>::MakePath(PathArray& path, unsigned int step) const
{
	if (!m_EndNode)
		return false;
	size_t offset = path.size();
	if (step)
	{
		const PathfindingNode *next = m_EndNode;
		const PathfindingNode *prev = m_EndNode->parent;
		if (!prev)
			return false;
		do
		{
			const unsigned x = next->pos.x, y = next->pos.y;
			int dx = int(prev->pos.x - x);
			int dy = int(prev->pos.y - y);
			ASSERT(!dx || !dy || abs(dx) == abs(dy));
			const int steps = std::max(abs(dx), abs(dy));
			dx /= std::max(abs(dx), 1);
			dy /= std::max(abs(dy), 1);
			dx *= int(step);
			dy *= int(step);
			int dxa = 0, dya = 0;
			for (int i = 0; i < steps; i += step)
			{
				path.push_back(Position(x + dxa, y + dya));
				dxa += dx;
				dya += dy;
			}
			next = prev;
			prev = prev->parent;
		} while (prev);
	}
	else
	{
		const PathfindingNode *next = m_EndNode;
		if (!next->parent)
			return false;
		do
		{
			ASSERT(next != next->parent);
			path.push_back(next->pos);
			next = next->parent;
		} while (next->parent);
	}
	std::reverse(path.begin() + offset, path.end());
	return true;
}

///////////////////////////////////////////////////////////////////////////////
// 길찾기 
///////////////////////////////////////////////////////////////////////////////
template <typename GRID, typename RULE>
bool BasePathFinder<GRID, RULE>::FindPath(PathArray& path, Position start, Position end, unsigned int step)
{
	// 길찾기에 필요한 초기화를 한다.
	ePATH_RESULT res = Init(start, end);

	// 경로가 없으면 true를 리턴하고 찾기가 실패되면 false를 리턴한다.
	if (res == PATH_RESULT_EMPTY_PATH)
		return true;

	int count = 0;

	while (true)
	{
		switch (res)
		{
		case PATH_RESULT_NEED_MORE_STEPS:
			res = NextStep();
			count++;
			break; // the switch
		case PATH_RESULT_FOUND_PATH:
			return Finish(path, step);
		case PATH_RESULT_NO_PATH:
		default:
			return false;
		}
	}
}

///////////////////////////////////////////////////////////////////////////////
// 길 찾기 초기화
///////////////////////////////////////////////////////////////////////////////
template <typename GRID, typename RULE>
ePATH_RESULT BasePathFinder<GRID, RULE>::Init(Position start, Position end)
{
	m_NodeSize = 0;
	m_OpenList.Clear();
	m_EndNode = nullptr;

	if (start == end)
	{
		// 끝이 갈수 있을 경우엔 갈수 있는 길로 셋팅한다.
		// 끝이 갈수 없은 경우 길이 없음
		return m_Grid(end.x, end.y) ? PATH_RESULT_EMPTY_PATH : PATH_RESULT_NO_PATH;
	}

	// 시작 위치와 끝 위치가 장애물이 있으면 PATH_RESULT_NO_PATH
	if (!m_Grid(start.x, start.y) || !m_Grid(end.x, end.y))
		return PATH_RESULT_NO_PATH;

	m_EndNode = GetNode(end);
	PathfindingNode *startNode = GetNode(start);
	ASSERT(startNode && m_EndNode);

	// 직선 길이 있으면 직선으로 밀어주기
	//if(FindStraight(startNode))
	//	return PATH_RESULT_FOUND_PATH;

	// 시작 노드을 오픈 노드에 넣어준다.
	m_OpenList.Push(startNode);

	return PATH_RESULT_NEED_MORE_STEPS;
}

///////////////////////////////////////////////////////////////////////////////
// 다음 스텝 길 찾기
///////////////////////////////////////////////////////////////////////////////
template <typename GRID, typename RULE>
ePATH_RESULT BasePathFinder<GRID, RULE>::NextStep()
{
	if (m_OpenList.Empty())
		return PATH_RESULT_NO_PATH;
	PathfindingNode *n = m_OpenList.Pop();
	n->SetClosed();
	if (n == m_EndNode)
		return PATH_RESULT_FOUND_PATH;
	IdentifySuccessors(n);
	return PATH_RESULT_NEED_MORE_STEPS;
}

///////////////////////////////////////////////////////////////////////////////
// 종료 하기 - 길 만들어 내기
///////////////////////////////////////////////////////////////////////////////
template<typename GRID, typename RULE>
bool BasePathFinder<GRID, RULE>::Finish(PathArray& path, unsigned int step)
{
	return MakePath(path, step);
}

///////////////////////////////////////////////////////////////////////////////
// 직선 길찾기
///////////////////////////////////////////////////////////////////////////////
template<typename GRID, typename RULE>
bool BasePathFinder<GRID, RULE>::FindStraight(PathfindingNode *n)
{
	Position midpos = npos;
	int x = n->pos.x;
	int y = n->pos.y;
	int ex = m_EndNode->pos.x;
	int ey = m_EndNode->pos.y;

	// 시작과 끝이 달라야한다. 
	ASSERT(x != ex || y != ey);
	ASSERT(n != m_EndNode);

	int dx = int(ex - x);
	int dy = int(ey - y);
	const int adx = abs(dx);
	const int ady = abs(dy);
	dx /= std::max(adx, 1);
	dy /= std::max(ady, 1);
	dx *= 1;
	dy *= 1;

	// 기울기가 있는 경우(대각선)
	if (x != ex && y != ey)
	{
		ASSERT(dx && dy);
		const int minlen = (int)std::min(adx, ady);
		const int tx = x + dx * minlen;
		for (; x != tx; )
		{
			if (m_Grid(x, y) && (m_Grid(x + dx, y) || m_Grid(x, y + dy)))
			{
				x += dx;
				y += dy;
			}
			else
				return false;
		}

		if (!m_Grid(x, y))
			return false;

		midpos = Pos(x, y);
	}

	// 한쪽 축으로만 체크하기
	ASSERT(x == ex || y == ey);

	if (!(x == ex && y == ey))
	{
		while (x != ex)
			if (!m_Grid(x += dx, y))
				return false;

		while (y != ey)
			if (!m_Grid(x, y += dy))
				return false;

		ASSERT(x == ex && y == ey);
	}

	if (midpos.isValid())
	{
		PathfindingNode *mid = GetNode(midpos);
		ASSERT(mid && mid != n);
		mid->parent = n;
		if (mid != m_EndNode)
			m_EndNode->parent = mid;
	}
	else
		m_EndNode->parent = n;

	return true;
}

}

#endif


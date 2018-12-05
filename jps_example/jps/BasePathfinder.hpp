#ifndef PATHFINDER_H
#define PATHFINDER_H

namespace JPS
{
// ��ã�� ���
enum ePATH_RESULT
{
	PATH_RESULT_NO_PATH,
	PATH_RESULT_FOUND_PATH,
	PATH_RESULT_NEED_MORE_STEPS,
	PATH_RESULT_EMPTY_PATH
};

// ��ã�� 
template <typename GRID, typename RULE>
class BasePathFinder
{
	const GRID& m_Grid;		// �׸���(�� ����)
	RULE m_Rule;
	PathfindingNode* m_EndNode;		// ������ ���
	UnsortedPriorityQueue m_OpenList;	// �� ����Ʈ
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
	// ���� ã��
	bool FindPath(PathArray& path, Position start, Position end, unsigned int Step = 0);
	// �ʱ�ȭ �ϱ�
	ePATH_RESULT Init(Position start, Position end);
	// �� step �� �� ã��
	ePATH_RESULT NextStep();
	// ���� �ϱ� - �� ����� ���� 
	bool Finish(PathArray& path, unsigned int step);
	
	size_t GetNodesExpanded() const { return m_NodeSize; }

private:
	// ��� ������ ������ �´�.
	PathfindingNode *GetNode(const Position& pos);
	// �ĺ��ڵ��� Ȯ���� ������. - �ֺ��̿��� ã�� Ž���Ѵ�.
	void IdentifySuccessors(const PathfindingNode *n);
	// ã�� ���� ������ ������� ���� �����.
	bool MakePath(PathArray& path, unsigned int step) const;
	// ���� ��� ã��
	bool FindStraight(PathfindingNode *start);
};


// ��� ���ϱ�(����)
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
// ���� ���� (a*)
///////////////////////////////////////////////////////////////////////////////
template <typename GRID, typename RULE>
void BasePathFinder<GRID, RULE>::IdentifySuccessors(const PathfindingNode *n)
{
	// �ֺ� 8���⿡ ���� Pos
	Position PosBuffer[8];

	const unsigned int num = m_Rule.FillNeighbors(n, &PosBuffer[0]);
	for (int i = num - 1; i >= 0; --i)
	{
		Position MovePos = m_Rule.Jump(PosBuffer[i], n->pos, m_EndNode->pos);

		if (!MovePos.isValid())
			continue;

		// forced neighbours �� ã�Ҵ�.
		// ���� MovePos ��ġ�� ��ȿ�� ������ ��ġ �̹Ƿ� ���� ��带 �������մϴ�.
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
// ã�� ���� ������ ������� ���� �����.
// m_EndNode �������� parent�� ã�� ���鼭 ���� ����
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
// ��ã�� 
///////////////////////////////////////////////////////////////////////////////
template <typename GRID, typename RULE>
bool BasePathFinder<GRID, RULE>::FindPath(PathArray& path, Position start, Position end, unsigned int step)
{
	// ��ã�⿡ �ʿ��� �ʱ�ȭ�� �Ѵ�.
	ePATH_RESULT res = Init(start, end);

	// ��ΰ� ������ true�� �����ϰ� ã�Ⱑ ���еǸ� false�� �����Ѵ�.
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
// �� ã�� �ʱ�ȭ
///////////////////////////////////////////////////////////////////////////////
template <typename GRID, typename RULE>
ePATH_RESULT BasePathFinder<GRID, RULE>::Init(Position start, Position end)
{
	m_NodeSize = 0;
	m_OpenList.Clear();
	m_EndNode = nullptr;

	if (start == end)
	{
		// ���� ���� ���� ��쿣 ���� �ִ� ��� �����Ѵ�.
		// ���� ���� ���� ��� ���� ����
		return m_Grid(end.x, end.y) ? PATH_RESULT_EMPTY_PATH : PATH_RESULT_NO_PATH;
	}

	// ���� ��ġ�� �� ��ġ�� ��ֹ��� ������ PATH_RESULT_NO_PATH
	if (!m_Grid(start.x, start.y) || !m_Grid(end.x, end.y))
		return PATH_RESULT_NO_PATH;

	m_EndNode = GetNode(end);
	PathfindingNode *startNode = GetNode(start);
	ASSERT(startNode && m_EndNode);

	// ���� ���� ������ �������� �о��ֱ�
	//if(FindStraight(startNode))
	//	return PATH_RESULT_FOUND_PATH;

	// ���� ����� ���� ��忡 �־��ش�.
	m_OpenList.Push(startNode);

	return PATH_RESULT_NEED_MORE_STEPS;
}

///////////////////////////////////////////////////////////////////////////////
// ���� ���� �� ã��
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
// ���� �ϱ� - �� ����� ����
///////////////////////////////////////////////////////////////////////////////
template<typename GRID, typename RULE>
bool BasePathFinder<GRID, RULE>::Finish(PathArray& path, unsigned int step)
{
	return MakePath(path, step);
}

///////////////////////////////////////////////////////////////////////////////
// ���� ��ã��
///////////////////////////////////////////////////////////////////////////////
template<typename GRID, typename RULE>
bool BasePathFinder<GRID, RULE>::FindStraight(PathfindingNode *n)
{
	Position midpos = npos;
	int x = n->pos.x;
	int y = n->pos.y;
	int ex = m_EndNode->pos.x;
	int ey = m_EndNode->pos.y;

	// ���۰� ���� �޶���Ѵ�. 
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

	// ���Ⱑ �ִ� ���(�밢��)
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

	// ���� �����θ� üũ�ϱ�
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


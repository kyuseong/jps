#ifndef PATHFINDRULE_JPS_H
#define PATHFINDRULE_JPS_H

namespace JPS
{
namespace Rule
{

// ��ã�� �� (JPS)
template <typename GRID>
class PathFindRule_JPS
{
protected:
	const GRID& m_Grid;		// �׸���(�� ����)
public:
	PathFindRule_JPS(const GRID& g) : m_Grid(g)	{}
	unsigned int FillNeighbors(const PathfindingNode *n, Position *wptr) const;
	virtual Position Jump(const Position& p, const Position& src, const Position& EndPos);
	void ChangedGrid(Position Start, Position End) {}

protected:
	virtual Position JumpD(Position p, int dx, int dy, const Position& EndPos);
	virtual Position JumpX(Position p, int dx, const Position& EndPos);
	virtual Position JumpY(Position p, int dy, const Position& EndPos);

	
	Position ComputeJumpD(Position p, int dx, int dy, const Position& EndPos, bool& ReachEndPos);
	Position ComputeJumpX(Position p, int dx, const Position& EndPos, bool& ReachEndPos);
	Position ComputeJumpY(Position p, int dy, const Position& EndPos, bool& ReachEndPos);
};


#define CHECKGRID(dx, dy) (m_Grid(x+(dx), y+(dy)))
#define ADDPOS(dx, dy) 	do { *w++ = Position(x+(dx), y+(dy)); } while(0)
#define ADDPOS_CHECK(dx, dy) do { if(CHECKGRID(dx, dy)) ADDPOS(dx, dy); } while(0)
#define ADDPOS_NO_TUNNEL(dx, dy) do { if(m_Grid(x+(dx),y) || m_Grid(x,y+(dy))) ADDPOS_CHECK(dx, dy); } while(0)

template <typename GRID>
unsigned int PathFindRule_JPS<GRID>::FillNeighbors(const PathfindingNode *n, Position *wptr) const
{
	Position *w = wptr;
	const unsigned int x = n->pos.x;
	const unsigned int y = n->pos.y;

	if (!n->parent)
	{
		// ��������
		ADDPOS_CHECK(-1, 0);
		ADDPOS_CHECK(0, -1);
		ADDPOS_CHECK(0, 1);
		ADDPOS_CHECK(1, 0);

		// �밢��
		ADDPOS_NO_TUNNEL(-1, -1);
		ADDPOS_NO_TUNNEL(-1, 1);
		ADDPOS_NO_TUNNEL(1, -1);
		ADDPOS_NO_TUNNEL(1, 1);

		return unsigned(w - wptr);
	}

	// ���� ����(-1, 0, or 1)
	int dx = int(x - n->parent->pos.x);
	dx /= std::max(abs(dx), 1);
	dx *= 1;
	int dy = int(y - n->parent->pos.y);
	dy /= std::max(abs(dy), 1);
	dy *= 1;

	if (dx && dy)
	{
		// �밢��(�̿�)
		bool walkX = false;
		bool walkY = false;
		if ((walkX = m_Grid(x + dx, y)))
			*w++ = Position(x + dx, y);
		if ((walkY = m_Grid(x, y + dy)))
			*w++ = Position(x, y + dy);

		if (walkX || walkY)
			ADDPOS_CHECK(dx, dy);

		// ���� �̿�
		if (walkY && !CHECKGRID(-dx, 0))
			ADDPOS_CHECK(-dx, dy);

		if (walkX && !CHECKGRID(0, -dy))
			ADDPOS_CHECK(dx, -dy);

	}
	else if (dx)
	{
		// X�� �̵�
		if (CHECKGRID(dx, 0))
		{
			ADDPOS(dx, 0);

			// ���� �̿�
			if (!CHECKGRID(0, 1))
				ADDPOS_CHECK(dx, 1);
			if (!CHECKGRID(0, -1))
				ADDPOS_CHECK(dx, -1);
		}


	}
	else if (dy)
	{
		// Y�� �̵�
		if (CHECKGRID(0, dy))
		{
			ADDPOS(0, dy);

			// ���� �̿�
			if (!CHECKGRID(1, 0))
				ADDPOS_CHECK(1, dy);
			if (!CHECKGRID(-1, 0))
				ADDPOS_CHECK(-1, dy);
		}
	}

	return (unsigned int)(w - wptr);
}


///////////////////////////////////////////////////////////////////////////////
// �����Ѵ�. (src�� p�� ���迡 ����)
///////////////////////////////////////////////////////////////////////////////
template <typename GRID>
Position PathFindRule_JPS<GRID>::Jump(const Position &p, const Position& src, const Position& EndPos)
{
	//ASSERT(m_Grid(p.x, p.y));
	int dx = int(p.x - src.x);
	int dy = int(p.y - src.y);
	ASSERT(dx || dy);

	if (dx && dy) {
		auto ret = JumpD(p, dx, dy, EndPos);
		return ret;
	}
	else if (dx) {
		auto ret = JumpX(p, dx, EndPos);
		return ret;
	}
	else if (dy) {
		auto ret = JumpY(p, dy, EndPos);
		return ret;
	}

	ASSERT(false);
	return npos;
}

///////////////////////////////////////////////////////////////////////////////
// Jump (�밢��)
///////////////////////////////////////////////////////////////////////////////
template <typename GRID>
Position PathFindRule_JPS<GRID>::JumpD(Position p, int dx, int dy, const Position& EndPos)
{
	bool ReachEndPos;
	return ComputeJumpD(p, dx, dy, EndPos, ReachEndPos);
}

///////////////////////////////////////////////////////////////////////////////
// Jump (�밢��)
///////////////////////////////////////////////////////////////////////////////
template <typename GRID>
Position PathFindRule_JPS<GRID>::ComputeJumpD(Position p, int dx, int dy, const Position& EndPos, bool& ReachEndPos)
{
	ReachEndPos = false;
	while (true)
	{
		if (p == EndPos) {
			ReachEndPos = true;
			break;
		}

		const unsigned int x = p.x;
		const unsigned int y = p.y;

		// �Ʒ� ó�� �浹�ϴ��� üũ�ϱ�
		//   ����
		// ��
		// ��

		if ((m_Grid(x - dx, y + dy) && !m_Grid(x - dx, y)) || (m_Grid(x + dx, y - dy) && !m_Grid(x, y - dy)))
			break;

		//   ����
		// ��  ��
		// ��
		// x �� �������� ��� ��ǥ�� �浹�߳� üũ�ϱ�
		const bool gdx = m_Grid(x + dx, y);

		// x ������ �޿� ���� ���� ���� üũ�Ѵ�.
		bool DummyX = false;
		if (gdx && JumpX(Position(x + dx, y), dx, EndPos).isValid())
		//if (gdx && ComputeJumpX(Position(x + dx, y), dx, EndPos, DummyX).isValid())
			break;

		//   ����
		// ��  
		// ����
		const bool gdy = m_Grid(x, y + dy);

		// y ������ �޿� ���� ���� ���� üũ�Ѵ�.
		bool DummyY = false;
		if (gdy && JumpY(Position(x, y + dy), dy, EndPos).isValid())
		//if (gdy && ComputeJumpY(Position(x, y + dy), dy, EndPos, DummyY).isValid())
			break;

		// 
		if ((gdx || gdy) && m_Grid(x + dx, y + dy))
		{
			// ���� �Ѵ�.
			p.x += dx;
			p.y += dy;
		}
		else
		{
			// ��� ��ǥ�� ������ �ٴ����� ���� ��������.
			p = npos;
			break;
		}
	}

	return p;
}

///////////////////////////////////////////////////////////////////////////////
// Jump X��
///////////////////////////////////////////////////////////////////////////////
template <typename GRID>
Position PathFindRule_JPS<GRID>::JumpX(Position p, int dx, const Position& EndPos)
{
	bool ReachEndPos;
	return ComputeJumpX(p, dx, EndPos, ReachEndPos);
}

///////////////////////////////////////////////////////////////////////////////
// Jump X��
///////////////////////////////////////////////////////////////////////////////
template <typename GRID>
Position PathFindRule_JPS<GRID>::ComputeJumpX(Position p, int dx, const Position& EndPos, bool& ReachEndPos)
{
	// �ٲ��� ���� y
	const unsigned int y = p.y;
	ReachEndPos = false;

	// ���� ��ǥ p �� ��, �Ʒ� ��ǥ�� ���ؼ� üũ�Ѵ�.
	unsigned int a = ~((unsigned int)(m_Grid(p.x, y + 1)) | ((unsigned int)(m_Grid(p.x, y - 1)) << 1));
	while (true)
	{
		// x ������ �����Ѵ�. 
		const unsigned int xx = p.x + dx;
		const unsigned int b = (unsigned int)(m_Grid(xx, y + 1)) | (unsigned int)((m_Grid(xx, y - 1)) << 1);

		// �ٱ� �ΰ��� ��ǥ�� �밢�� ������ ��ǥ�� �浹 ��ǥ�� 
		// ���̰ų�
		if ((b & a))
			break;

		if (p == EndPos) {
			ReachEndPos = true;
			break;
		}

		// ���� �ε�����.
		if (!m_Grid(xx, y))
		{
			// ������ ������
			p = npos;
			break;
		}

		p.x += dx;
		a = ~b;
	}

	return p;
}

///////////////////////////////////////////////////////////////////////////////
// Jump Y��
///////////////////////////////////////////////////////////////////////////////
template <typename GRID>
Position PathFindRule_JPS<GRID>::JumpY(Position p, int dy, const Position& EndPos)
{
	bool ReachEndPos;
	return ComputeJumpY(p, dy, EndPos, ReachEndPos);
}

///////////////////////////////////////////////////////////////////////////////
// Jump Y��
///////////////////////////////////////////////////////////////////////////////
template <typename GRID>
Position PathFindRule_JPS<GRID>::ComputeJumpY(Position p, int dy, const Position& EndPos, bool& ReachEndPos)
{
	ReachEndPos = false;
	// x �� ����
	const unsigned int x = p.x;

	unsigned int a = ~((unsigned int)(m_Grid(x + 1, p.y)) | ((unsigned int)(m_Grid(x - 1, p.y)) << 1));

	while (true)
	{
		const unsigned int yy = p.y + dy;
		const unsigned int b = (unsigned int)(m_Grid(x + 1, yy)) | (unsigned int)((m_Grid(x - 1, yy)) << 1);
		// �ٱ� �ΰ��� ��ǥ�� �밢�� ������ ��ǥ�� �浹 ��ǥ�� 
		// ���̰ų�
		if (a & b)
			break;
		if (p == EndPos) {
			ReachEndPos = true;
			break;
		}

		// �浹ü
		if (!m_Grid(x, yy))
		{
			p = npos;
			break;
		}

		p.y += dy;
		a = ~b;
	}

	return p;
}



}
}


#endif
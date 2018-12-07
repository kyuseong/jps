#ifndef PATHFINDRULE_JPS_H
#define PATHFINDRULE_JPS_H

namespace JPS
{
namespace Rule
{

// 길찾기 룰 (JPS)
template <typename GRID>
class PathFindRule_JPS
{
protected:
	const GRID& m_Grid;		// 그리드(맵 정보)
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
		// 동서남북
		ADDPOS_CHECK(-1, 0);
		ADDPOS_CHECK(0, -1);
		ADDPOS_CHECK(0, 1);
		ADDPOS_CHECK(1, 0);

		// 대각선
		ADDPOS_NO_TUNNEL(-1, -1);
		ADDPOS_NO_TUNNEL(-1, 1);
		ADDPOS_NO_TUNNEL(1, -1);
		ADDPOS_NO_TUNNEL(1, 1);

		return unsigned(w - wptr);
	}

	// 방향 점프(-1, 0, or 1)
	int dx = int(x - n->parent->pos.x);
	dx /= std::max(abs(dx), 1);
	dx *= 1;
	int dy = int(y - n->parent->pos.y);
	dy /= std::max(abs(dy), 1);
	dy *= 1;

	if (dx && dy)
	{
		// 대각선(이웃)
		bool walkX = false;
		bool walkY = false;
		if ((walkX = m_Grid(x + dx, y)))
			*w++ = Position(x + dx, y);
		if ((walkY = m_Grid(x, y + dy)))
			*w++ = Position(x, y + dy);

		if (walkX || walkY)
			ADDPOS_CHECK(dx, dy);

		// 강제 이웃
		if (walkY && !CHECKGRID(-dx, 0))
			ADDPOS_CHECK(-dx, dy);

		if (walkX && !CHECKGRID(0, -dy))
			ADDPOS_CHECK(dx, -dy);

	}
	else if (dx)
	{
		// X축 이동
		if (CHECKGRID(dx, 0))
		{
			ADDPOS(dx, 0);

			// 강제 이웃
			if (!CHECKGRID(0, 1))
				ADDPOS_CHECK(dx, 1);
			if (!CHECKGRID(0, -1))
				ADDPOS_CHECK(dx, -1);
		}


	}
	else if (dy)
	{
		// Y축 이동
		if (CHECKGRID(0, dy))
		{
			ADDPOS(0, dy);

			// 강제 이웃
			if (!CHECKGRID(1, 0))
				ADDPOS_CHECK(1, dy);
			if (!CHECKGRID(-1, 0))
				ADDPOS_CHECK(-1, dy);
		}
	}

	return (unsigned int)(w - wptr);
}


///////////////////////////////////////////////////////////////////////////////
// 점프한다. (src와 p의 관계에 따라서)
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
// Jump (대각선)
///////////////////////////////////////////////////////////////////////////////
template <typename GRID>
Position PathFindRule_JPS<GRID>::JumpD(Position p, int dx, int dy, const Position& EndPos)
{
	bool ReachEndPos;
	return ComputeJumpD(p, dx, dy, EndPos, ReachEndPos);
}

///////////////////////////////////////////////////////////////////////////////
// Jump (대각선)
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

		// 아래 처럼 충돌하는지 체크하기
		//   ㅇㅇ
		// ㅇ
		// ㅇ

		if ((m_Grid(x - dx, y + dy) && !m_Grid(x - dx, y)) || (m_Grid(x + dx, y - dy) && !m_Grid(x, y - dy)))
			break;

		//   ㅇㅇ
		// ㅇ  ㅇ
		// ㅇ
		// x 축 기준으로 가운데 좌표가 충돌했나 체크하기
		const bool gdx = m_Grid(x + dx, y);

		// x 축으로 쭈욱 점프 가능 한지 체크한다.
		bool DummyX = false;
		if (gdx && JumpX(Position(x + dx, y), dx, EndPos).isValid())
		//if (gdx && ComputeJumpX(Position(x + dx, y), dx, EndPos, DummyX).isValid())
			break;

		//   ㅇㅇ
		// ㅇ  
		// ㅇㅇ
		const bool gdy = m_Grid(x, y + dy);

		// y 축으로 쭈욱 점프 가능 한지 체크한다.
		bool DummyY = false;
		if (gdy && JumpY(Position(x, y + dy), dy, EndPos).isValid())
		//if (gdy && ComputeJumpY(Position(x, y + dy), dy, EndPos, DummyY).isValid())
			break;

		// 
		if ((gdx || gdy) && m_Grid(x + dx, y + dy))
		{
			// 전진 한다.
			p.x += dx;
			p.y += dy;
		}
		else
		{
			// 가운데 좌표가 끝까지 다달으면 관심 없어진다.
			p = npos;
			break;
		}
	}

	return p;
}

///////////////////////////////////////////////////////////////////////////////
// Jump X축
///////////////////////////////////////////////////////////////////////////////
template <typename GRID>
Position PathFindRule_JPS<GRID>::JumpX(Position p, int dx, const Position& EndPos)
{
	bool ReachEndPos;
	return ComputeJumpX(p, dx, EndPos, ReachEndPos);
}

///////////////////////////////////////////////////////////////////////////////
// Jump X축
///////////////////////////////////////////////////////////////////////////////
template <typename GRID>
Position PathFindRule_JPS<GRID>::ComputeJumpX(Position p, int dx, const Position& EndPos, bool& ReachEndPos)
{
	// 바뀌지 마라 y
	const unsigned int y = p.y;
	ReachEndPos = false;

	// 이전 좌표 p 의 위, 아래 좌표에 대해서 체크한다.
	unsigned int a = ~((unsigned int)(m_Grid(p.x, y + 1)) | ((unsigned int)(m_Grid(p.x, y - 1)) << 1));
	while (true)
	{
		// x 축으로 전진한다. 
		const unsigned int xx = p.x + dx;
		const unsigned int b = (unsigned int)(m_Grid(xx, y + 1)) | (unsigned int)((m_Grid(xx, y - 1)) << 1);

		// 바깥 두개의 좌표에 대각선 전진한 좌표가 충돌 좌표면 
		// 끝이거나
		if ((b & a))
			break;

		if (p == EndPos) {
			ReachEndPos = true;
			break;
		}

		// 벽에 부딪혔다.
		if (!m_Grid(xx, y))
		{
			// 관심이 없어짐
			p = npos;
			break;
		}

		p.x += dx;
		a = ~b;
	}

	return p;
}

///////////////////////////////////////////////////////////////////////////////
// Jump Y축
///////////////////////////////////////////////////////////////////////////////
template <typename GRID>
Position PathFindRule_JPS<GRID>::JumpY(Position p, int dy, const Position& EndPos)
{
	bool ReachEndPos;
	return ComputeJumpY(p, dy, EndPos, ReachEndPos);
}

///////////////////////////////////////////////////////////////////////////////
// Jump Y축
///////////////////////////////////////////////////////////////////////////////
template <typename GRID>
Position PathFindRule_JPS<GRID>::ComputeJumpY(Position p, int dy, const Position& EndPos, bool& ReachEndPos)
{
	ReachEndPos = false;
	// x 축 고정
	const unsigned int x = p.x;

	unsigned int a = ~((unsigned int)(m_Grid(x + 1, p.y)) | ((unsigned int)(m_Grid(x - 1, p.y)) << 1));

	while (true)
	{
		const unsigned int yy = p.y + dy;
		const unsigned int b = (unsigned int)(m_Grid(x + 1, yy)) | (unsigned int)((m_Grid(x - 1, yy)) << 1);
		// 바깥 두개의 좌표에 대각선 전진한 좌표가 충돌 좌표면 
		// 끝이거나
		if (a & b)
			break;
		if (p == EndPos) {
			ReachEndPos = true;
			break;
		}

		// 충돌체
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
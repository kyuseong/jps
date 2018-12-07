#ifndef PATHFINDRULE_JPS_PLUS_H
#define PATHFINDRULE_JPS_PLUS_H

namespace JPS
{
namespace Rule
{

// 길찾기 룰 (JPS+ 스태틱 맵=변경되지 않음)
template <typename GRID, bool PRECOMPUTE>
class PathFindRule_JPSPlus : public PathFindRule_JPS<GRID>
{
	const int JUMP_DIR = 8;

	const int m_Height;		// 맵의 높이
	const int m_Width;		// 맵의 넓이
	Position* m_PreComputeTable;
public:
	PathFindRule_JPSPlus(const GRID& g);
	~PathFindRule_JPSPlus();

protected:
	void InitMap();
	void PrecomputeMap();
	
	virtual Position JumpD(Position p, int dx, int dy, const Position& EndPos);
	virtual Position JumpX(Position p, int dx, const Position& EndPos);
	virtual Position JumpY(Position p, int dy, const Position& EndPos);

	bool HasPreComputeJumpPoint(const Position& p, int dx, int dy) const;
	Position GetPreComputeJumpPoint(const Position& p, int dx, int dy) const;
	void SetPreComputeJumpPoint(const Position& p, int dx, int dy, const Position& JumpPos);
};

///////////////////////////////////////////////////////////////////////////////
// ctor
///////////////////////////////////////////////////////////////////////////////
template <typename GRID, bool PRECOMPUTE>
PathFindRule_JPSPlus<GRID, PRECOMPUTE>::PathFindRule_JPSPlus(const GRID& g)
	: PathFindRule_JPS(g), m_Height(g.GetHeight()), m_Width(g.GetWidth())
{
	InitMap();

	if (PRECOMPUTE)
	{
		PrecomputeMap();
	}
}
///////////////////////////////////////////////////////////////////////////////
// dtor
///////////////////////////////////////////////////////////////////////////////
template <typename GRID, bool PRECOMPUTE>
PathFindRule_JPSPlus<GRID, PRECOMPUTE>::~PathFindRule_JPSPlus()
{
	delete[] m_PreComputeTable;
}

///////////////////////////////////////////////////////////////////////////////
// 초기화 (맵)
///////////////////////////////////////////////////////////////////////////////
template <typename GRID, bool PRECOMPUTE>
void PathFindRule_JPSPlus<GRID, PRECOMPUTE>::InitMap()
{
	m_PreComputeTable = new Position[m_Height*m_Width*JUMP_DIR];
	for (int h = 0; h < m_Height; ++h) {
		for (int w = 0; w < m_Width; ++w) {
			for (int k = 0; k < JUMP_DIR; ++k) {
				m_PreComputeTable[h*m_Width*JUMP_DIR + w * JUMP_DIR + k].x = 0;
			}
		}
	}
}
///////////////////////////////////////////////////////////////////////////////
// 전처리 (맵)
///////////////////////////////////////////////////////////////////////////////
template <typename GRID, bool PRECOMPUTE>
void PathFindRule_JPSPlus<GRID, PRECOMPUTE>::PrecomputeMap()
{
	bool End = false;

	// 직선 먼저 테이블을 만든다.
	for (int y = 0; y < m_Height; ++y) {
		for (int x = 0; x < m_Width; ++x) {
			if (m_Grid(x, y))
			{
				m_PreComputeTable[y * m_Width * JUMP_DIR + x * JUMP_DIR + 2] = ComputeJumpX(Position(x, y), 1, JPS::npos, End);
				m_PreComputeTable[y * m_Width * JUMP_DIR + x * JUMP_DIR + 5] = ComputeJumpX(Position(x, y), -1, JPS::npos, End);

				m_PreComputeTable[y * m_Width * JUMP_DIR + x * JUMP_DIR + 6] = ComputeJumpY(Position(x, y), 1, JPS::npos, End);
				m_PreComputeTable[y * m_Width * JUMP_DIR + x * JUMP_DIR + 7] = ComputeJumpY(Position(x, y), -1, JPS::npos, End);
			}
			else
			{
				// 못가면 그냥 -1
				for (int k = 0; k < JUMP_DIR; ++k) {
					m_PreComputeTable[y * m_Width * JUMP_DIR + x * JUMP_DIR + 0].x = -1;
				}
			}
		}
	}
	// 대각선 테이블 만든다.(위에 직선 테이블 사용하도록)
	for (int y = 0; y < m_Height; ++y) {
		for (int x = 0; x < m_Width; ++x) {
			// 갈수 잇는 곳이면 8방에 해당하는 값을 구한다.
			if (m_Grid(x, y))
			{
				m_PreComputeTable[y * m_Width * JUMP_DIR + x * JUMP_DIR + 0] = ComputeJumpD(Position(x, y), 1, 1, JPS::npos, End);
				m_PreComputeTable[y * m_Width * JUMP_DIR + x * JUMP_DIR + 1] = ComputeJumpD(Position(x, y), 1, -1, JPS::npos, End);

				m_PreComputeTable[y * m_Width * JUMP_DIR + x * JUMP_DIR + 3] = ComputeJumpD(Position(x, y), -1, 1, JPS::npos, End);
				m_PreComputeTable[y * m_Width * JUMP_DIR + x * JUMP_DIR + 4] = ComputeJumpD(Position(x, y), -1, -1, JPS::npos, End);
			}
		}
	}
}

///////////////////////////////////////////////////////////////////////////////
// Jump (대각선)
///////////////////////////////////////////////////////////////////////////////
template <typename GRID, bool PRECOMPUTE>
Position PathFindRule_JPSPlus<GRID, PRECOMPUTE>::JumpD(Position p, int dx, int dy, const Position& EndPos)
{
	if (EndPos.isValid())
	{
		if (dx > 0)
		{
			if (dy > 0)
			{
				if (p.x <= EndPos.x && p.y <= EndPos.y)
				{
					// 무조건 테이블 참조하지 않음
				}
				else
				{
					if (HasPreComputeJumpPoint(p, dx, dy))
						return GetPreComputeJumpPoint(p, dx, dy);
				}
			}
			else
			{
				if (p.x <= EndPos.x && p.y >= EndPos.y)
				{
					// 무조건 테이블 참조하지 않음
				}
				else
				{
					if (HasPreComputeJumpPoint(p, dx, dy))
						return GetPreComputeJumpPoint(p, dx, dy);
				}
			}
		}
		else
		{
			if (dy > 0)
			{
				if (p.x >= EndPos.x && p.y <= EndPos.y)
				{
					// 무조건 테이블 참조하지 않음
				}
				else
				{
					if (HasPreComputeJumpPoint(p, dx, dy))
						return GetPreComputeJumpPoint(p, dx, dy);
				}
			}
			else
			{
				if (p.x >= EndPos.x && p.y >= EndPos.y)
				{
					// 무조건 테이블 참조하지 않음
				}
				else
				{
					if (HasPreComputeJumpPoint(p, dx, dy))
						return GetPreComputeJumpPoint(p, dx, dy);
				}
			}
		}
	}
	const auto Start = p;

	bool ReachEndPos = false;

	p = ComputeJumpD(p, dx, dy, EndPos, ReachEndPos);

	if (ReachEndPos == false)
		SetPreComputeJumpPoint(Start, dx, dy, p);

	return p;
}

///////////////////////////////////////////////////////////////////////////////
// Jump X축
///////////////////////////////////////////////////////////////////////////////
template <typename GRID, bool PRECOMPUTE>
Position PathFindRule_JPSPlus<GRID, PRECOMPUTE>::JumpX(Position p, int dx, const Position& EndPos)
{
	//ASSERT(dx);
	//ASSERT(m_Grid(p.x, p.y));
	if (p.y != EndPos.y && HasPreComputeJumpPoint(p, dx, 0))
		return GetPreComputeJumpPoint(p, dx, 0);

	const auto Start = p;
	
	bool ReachEndPos = false;

	p = ComputeJumpX(p, dx, EndPos, ReachEndPos);

	if (ReachEndPos == false)
		SetPreComputeJumpPoint(Start, dx, 0, p);

	return p;
}

///////////////////////////////////////////////////////////////////////////////
// Jump Y축
///////////////////////////////////////////////////////////////////////////////
template <typename GRID, bool PRECOMPUTE>
Position PathFindRule_JPSPlus<GRID, PRECOMPUTE>::JumpY(Position p, int dy, const Position& EndPos)
{
	//ASSERT(dy);
	//ASSERT(m_Grid(p.x, p.y));
	if (p.x != EndPos.x && HasPreComputeJumpPoint(p, 0, dy))
		return GetPreComputeJumpPoint(p, 0, dy);

	const auto Start = p;

	bool ReachEndPos = false;
	p = ComputeJumpY(p, dy, EndPos, ReachEndPos);	

	if (ReachEndPos == false) {
		SetPreComputeJumpPoint(Start, 0, dy, p);
	}

	return p;
}

template <typename GRID, bool PRECOMPUTE>
bool PathFindRule_JPSPlus<GRID, PRECOMPUTE>::HasPreComputeJumpPoint(const Position& p, int dx, int dy) const
{
	if (dx > 0) {
		if (dy > 0) {
			return m_PreComputeTable[p.y * m_Width * JUMP_DIR + p.x * JUMP_DIR + 0].x != 0;
		}
		else if (dy < 0)
		{
			return m_PreComputeTable[p.y * m_Width * JUMP_DIR + p.x * JUMP_DIR + 1].x != 0;
		}
		else
		{
			return m_PreComputeTable[p.y * m_Width * JUMP_DIR + p.x * JUMP_DIR + 2].x != 0;
		}
	}
	else if (dx < 0)
	{
		if (dy > 0) {
			return m_PreComputeTable[p.y * m_Width * JUMP_DIR + p.x * JUMP_DIR + 3].x != 0;
		}
		else if (dy < 0)
		{
			return m_PreComputeTable[p.y * m_Width * JUMP_DIR + p.x * JUMP_DIR + 4].x != 0;
		}
		else
		{
			return m_PreComputeTable[p.y * m_Width * JUMP_DIR + p.x * JUMP_DIR + 5].x != 0;
		}
	}
	else
	{
		if (dy > 0) {
			return m_PreComputeTable[p.y * m_Width * JUMP_DIR + p.x * JUMP_DIR + 6].x != 0;
		}
		else if (dy < 0)
		{
			return m_PreComputeTable[p.y * m_Width * JUMP_DIR + p.x * JUMP_DIR + 7].x != 0;
		}
		else
		{
			ASSERT(false);
			return false;
		}
	}
}

template <typename GRID, bool PRECOMPUTE>
Position PathFindRule_JPSPlus<GRID, PRECOMPUTE>::GetPreComputeJumpPoint(const Position& p, int dx, int dy) const
{
	if (dx > 0) {
		if (dy > 0) {
			return m_PreComputeTable[p.y * m_Width * JUMP_DIR + p.x * JUMP_DIR + 0];
		}
		else if (dy < 0)
		{
			return m_PreComputeTable[p.y * m_Width * JUMP_DIR + p.x * JUMP_DIR + 1];
		}
		else
		{
			return m_PreComputeTable[p.y * m_Width * JUMP_DIR + p.x * JUMP_DIR + 2];
		}
	}
	else if (dx < 0)
	{
		if (dy > 0) {
			return m_PreComputeTable[p.y * m_Width * JUMP_DIR + p.x * JUMP_DIR + 3];
		}
		else if (dy < 0)
		{
			return m_PreComputeTable[p.y * m_Width * JUMP_DIR + p.x * JUMP_DIR + 4];
		}
		else
		{
			return m_PreComputeTable[p.y * m_Width * JUMP_DIR + p.x * JUMP_DIR + 5];
		}
	}
	else
	{
		if (dy > 0) {
			return m_PreComputeTable[p.y * m_Width * JUMP_DIR + p.x * JUMP_DIR + 6];
		}
		else if (dy < 0)
		{
			return m_PreComputeTable[p.y * m_Width * JUMP_DIR + p.x * JUMP_DIR + 7];
		}
		else
		{
			ASSERT(false);
			return Position();
		}
	}
}

template <typename GRID, bool PRECOMPUTE>
void PathFindRule_JPSPlus<GRID, PRECOMPUTE>::SetPreComputeJumpPoint(const Position& p, int dx, int dy, const Position& JumpPos)
{
	if (dx > 0) {
		if (dy > 0) {
			m_PreComputeTable[p.y * m_Width * JUMP_DIR + p.x * JUMP_DIR + 0] = JumpPos;
		}
		else if (dy < 0)
		{
			m_PreComputeTable[p.y * m_Width * JUMP_DIR + p.x * JUMP_DIR + 1] = JumpPos;
		}
		else
		{
			m_PreComputeTable[p.y * m_Width * JUMP_DIR + p.x * JUMP_DIR + 2] = JumpPos;
		}
	}
	else if (dx < 0)
	{
		if (dy > 0) {
			m_PreComputeTable[p.y * m_Width * JUMP_DIR + p.x * JUMP_DIR + 3] = JumpPos;
		}
		else if (dy < 0)
		{
			m_PreComputeTable[p.y * m_Width * JUMP_DIR + p.x * JUMP_DIR + 4] = JumpPos;
		}
		else
		{
			m_PreComputeTable[p.y * m_Width * JUMP_DIR + p.x * JUMP_DIR + 5] = JumpPos;
		}
	}
	else
	{
		if (dy > 0) {
			m_PreComputeTable[p.y * m_Width * JUMP_DIR + p.x * JUMP_DIR + 6] = JumpPos;
		}
		else if (dy < 0)
		{
			m_PreComputeTable[p.y * m_Width * JUMP_DIR + p.x * JUMP_DIR + 7] = JumpPos;
		}
		else
		{
			ASSERT(false);
			return;
		}
	}
}


}
}

#endif
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
	Position* m_PreprocessTable;
public:
	using PathFindRule_JPS<GRID>::PathFindRule_JPS;

	PathFindRule_JPSPlus(const GRID& g)
		: PathFindRule_JPS(g), m_Height(g.GetHeight()), m_Width(g.GetWidth())
	{
		InitMap();

		if (PRECOMPUTE)
		{
			PrecomputeMap();
		}
	}
	~PathFindRule_JPSPlus()
	{
		delete[] m_PreprocessTable; 
	}

	void InitMap()
	{
		m_PreprocessTable = new Position[m_Height*m_Width*JUMP_DIR];
		for (int h = 0; h < m_Height; ++h) {
			for (int w = 0; w < m_Width; ++w) {
				for (int k = 0; k < JUMP_DIR; ++k) {
					m_PreprocessTable[h*m_Width*JUMP_DIR + w * JUMP_DIR + k].x = 0;
				}
			}
		}
	}
	void PrecomputeMap()
	{
		for (int y = 0; y < m_Height; ++y) {
			for (int x = 0; x < m_Width; ++x) {
				// 갈수 잇는 곳이면 8방에 해당하는 값을 구한다.
				if (m_Grid(x, y))
				{
					bool End = false;
					m_PreprocessTable[y * m_Width * JUMP_DIR + x * JUMP_DIR + 0] = ComputeJumpD(Position(x, y), 1, 1, JPS::npos, End);
					m_PreprocessTable[y * m_Width * JUMP_DIR + x * JUMP_DIR + 1] = ComputeJumpD(Position(x, y), 1, -1, JPS::npos, End);
					m_PreprocessTable[y * m_Width * JUMP_DIR + x * JUMP_DIR + 2] = ComputeJumpX(Position(x, y), 1, JPS::npos, End);

					m_PreprocessTable[y * m_Width * JUMP_DIR + x * JUMP_DIR + 3] = ComputeJumpD(Position(x, y), -1, 1, JPS::npos, End);
					m_PreprocessTable[y * m_Width * JUMP_DIR + x * JUMP_DIR + 4] = ComputeJumpD(Position(x, y), -1, -1, JPS::npos, End);
					m_PreprocessTable[y * m_Width * JUMP_DIR + x * JUMP_DIR + 5] = ComputeJumpX(Position(x, y), -1,JPS::npos, End);

					m_PreprocessTable[y * m_Width * JUMP_DIR + x * JUMP_DIR + 6] = ComputeJumpY(Position(x, y), 1, JPS::npos, End);
					m_PreprocessTable[y * m_Width * JUMP_DIR + x * JUMP_DIR + 7] = ComputeJumpY(Position(x, y), -1, JPS::npos, End);
				}
				else
				{
					// 못가면 그냥 -1
					for (int k = 0; k < JUMP_DIR; ++k) {
						m_PreprocessTable[y * m_Width * JUMP_DIR + x * JUMP_DIR + 0].x = -1;
					}
				}
			}
		}
	}
	
	virtual Position JumpD(Position p, int dx, int dy, const Position& EndPos);
	virtual Position JumpX(Position p, int dx, const Position& EndPos);
	virtual Position JumpY(Position p, int dy, const Position& EndPos);

	bool HasJumpPoint(const Position& p, int dx, int dy) const;
	Position GetJumpPoint(const Position& p, int dx, int dy) const;
	void SetJumpPoint(const Position& p, int dx, int dy, const Position& JumpPos);
};

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
					if (HasJumpPoint(p, dx, dy))
						return GetJumpPoint(p, dx, dy);
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
					if (HasJumpPoint(p, dx, dy))
						return GetJumpPoint(p, dx, dy);
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
					if (HasJumpPoint(p, dx, dy))
						return GetJumpPoint(p, dx, dy);
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
					if (HasJumpPoint(p, dx, dy))
						return GetJumpPoint(p, dx, dy);
				}
			}
		}
	}
	const auto Start = p;

	bool ReachEndPos = false;

	p = ComputeJumpD(p, dx, dy, EndPos, ReachEndPos);

	if (ReachEndPos == false)
		SetJumpPoint(Start, dx, dy, p);

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
	if (p.y != EndPos.y && HasJumpPoint(p, dx, 0))
		return GetJumpPoint(p, dx, 0);

	const auto Start = p;
	
	bool ReachEndPos = false;

	p = ComputeJumpX(p, dx, EndPos, ReachEndPos);

	if (ReachEndPos == false)
		SetJumpPoint(Start, dx, 0, p);

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
	if (p.x != EndPos.x && HasJumpPoint(p, 0, dy))
		return GetJumpPoint(p, 0, dy);

	const auto Start = p;

	bool ReachEndPos = false;
	p = ComputeJumpY(p, dy, EndPos, ReachEndPos);	

	if (ReachEndPos == false) {
		SetJumpPoint(Start, 0, dy, p);
	}

	return p;
}

template <typename GRID, bool PRECOMPUTE>
bool PathFindRule_JPSPlus<GRID, PRECOMPUTE>::HasJumpPoint(const Position& p, int dx, int dy) const
{
	if (dx > 0) {
		if (dy > 0) {
			return m_PreprocessTable[p.y * m_Width * JUMP_DIR + p.x * JUMP_DIR + 0].x != 0;
		}
		else if (dy < 0)
		{
			return m_PreprocessTable[p.y * m_Width * JUMP_DIR + p.x * JUMP_DIR + 1].x != 0;
		}
		else
		{
			return m_PreprocessTable[p.y * m_Width * JUMP_DIR + p.x * JUMP_DIR + 2].x != 0;
		}
	}
	else if (dx < 0)
	{
		if (dy > 0) {
			return m_PreprocessTable[p.y * m_Width * JUMP_DIR + p.x * JUMP_DIR + 3].x != 0;
		}
		else if (dy < 0)
		{
			return m_PreprocessTable[p.y * m_Width * JUMP_DIR + p.x * JUMP_DIR + 4].x != 0;
		}
		else
		{
			return m_PreprocessTable[p.y * m_Width * JUMP_DIR + p.x * JUMP_DIR + 5].x != 0;
		}
	}
	else
	{
		if (dy > 0) {
			return m_PreprocessTable[p.y * m_Width * JUMP_DIR + p.x * JUMP_DIR + 6].x != 0;
		}
		else if (dy < 0)
		{
			return m_PreprocessTable[p.y * m_Width * JUMP_DIR + p.x * JUMP_DIR + 7].x != 0;
		}
		else
		{
			ASSERT(false);
			return false;
		}
	}
}

template <typename GRID, bool PRECOMPUTE>
Position PathFindRule_JPSPlus<GRID, PRECOMPUTE>::GetJumpPoint(const Position& p, int dx, int dy) const
{
	if (dx > 0) {
		if (dy > 0) {
			return m_PreprocessTable[p.y * m_Width * JUMP_DIR + p.x * JUMP_DIR + 0];
		}
		else if (dy < 0)
		{
			return m_PreprocessTable[p.y * m_Width * JUMP_DIR + p.x * JUMP_DIR + 1];
		}
		else
		{
			return m_PreprocessTable[p.y * m_Width * JUMP_DIR + p.x * JUMP_DIR + 2];
		}
	}
	else if (dx < 0)
	{
		if (dy > 0) {
			return m_PreprocessTable[p.y * m_Width * JUMP_DIR + p.x * JUMP_DIR + 3];
		}
		else if (dy < 0)
		{
			return m_PreprocessTable[p.y * m_Width * JUMP_DIR + p.x * JUMP_DIR + 4];
		}
		else
		{
			return m_PreprocessTable[p.y * m_Width * JUMP_DIR + p.x * JUMP_DIR + 5];
		}
	}
	else
	{
		if (dy > 0) {
			return m_PreprocessTable[p.y * m_Width * JUMP_DIR + p.x * JUMP_DIR + 6];
		}
		else if (dy < 0)
		{
			return m_PreprocessTable[p.y * m_Width * JUMP_DIR + p.x * JUMP_DIR + 7];
		}
		else
		{
			ASSERT(false);
			return Position();
		}
	}
}

template <typename GRID, bool PRECOMPUTE>
void PathFindRule_JPSPlus<GRID, PRECOMPUTE>::SetJumpPoint(const Position& p, int dx, int dy, const Position& JumpPos)
{
	if (dx > 0) {
		if (dy > 0) {
			m_PreprocessTable[p.y * m_Width * JUMP_DIR + p.x * JUMP_DIR + 0] = JumpPos;
		}
		else if (dy < 0)
		{
			m_PreprocessTable[p.y * m_Width * JUMP_DIR + p.x * JUMP_DIR + 1] = JumpPos;
		}
		else
		{
			m_PreprocessTable[p.y * m_Width * JUMP_DIR + p.x * JUMP_DIR + 2] = JumpPos;
		}
	}
	else if (dx < 0)
	{
		if (dy > 0) {
			m_PreprocessTable[p.y * m_Width * JUMP_DIR + p.x * JUMP_DIR + 3] = JumpPos;
		}
		else if (dy < 0)
		{
			m_PreprocessTable[p.y * m_Width * JUMP_DIR + p.x * JUMP_DIR + 4] = JumpPos;
		}
		else
		{
			m_PreprocessTable[p.y * m_Width * JUMP_DIR + p.x * JUMP_DIR + 5] = JumpPos;
		}
	}
	else
	{
		if (dy > 0) {
			m_PreprocessTable[p.y * m_Width * JUMP_DIR + p.x * JUMP_DIR + 6] = JumpPos;
		}
		else if (dy < 0)
		{
			m_PreprocessTable[p.y * m_Width * JUMP_DIR + p.x * JUMP_DIR + 7] = JumpPos;
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
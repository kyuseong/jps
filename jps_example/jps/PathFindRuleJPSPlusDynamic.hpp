#ifndef PATHFINDRULE_JPS_PLUS_DYNAMIC_H
#define PATHFINDRULE_JPS_PLUS_DYNAMIC_H

namespace JPS
{
namespace Rule
{

// 길찾기 룰 (JPS+ 다이나믹 맵=변경 될수 있는 맵)
// grid 의 정보가 변경될수 있을때 사용한다.
// 기본적으로 4방향 직선 포인트만 전처리를 사용하고 대각선 방향을 전처리를 하지 않는다.
// - 맵이 변경될 경우 너무 많은 대각선 방향의 포인트를 전처리 해야하므로 부하가 생긴다.
// - 대각선 포인트 Jump 시에 직선 전처리를 사용해서 jump를 하도록 해야한다.
// - 맵이 변경된 경우 전처리된 정보를 변경된 grid point +3,+3 만큼 충돌체가 없는 범위까지 테이블은 초기화 해야한다.
template <typename GRID, bool PRECOMPUTE>
class PathFindRule_JPSPlusDynamic : public PathFindRule_JPS<GRID>
{
	const int JUMP_DIR = 4;

	const int m_Height;		// 맵의 높이
	const int m_Width;		// 맵의 넓이
	Position* m_PreComputeTable;	// 전처리 맵
public:
	PathFindRule_JPSPlusDynamic(const GRID& g);
	~PathFindRule_JPSPlusDynamic();

	void ChangedGrid(Position Start, Position End);

protected:

	void InitMap();
	void PrecomputeMap();

	virtual Position JumpD(Position p, int dx, int dy, const Position& EndPos);
	virtual Position JumpX(Position p, int dx, const Position& EndPos);
	virtual Position JumpY(Position p, int dy, const Position& EndPos);

	bool HasPreComputeJumpPoint(const Position& p, int dx, int dy) const;
	Position GetPreComputeJumpPoint(const Position& p, int dx, int dy) const;
	void SetPreComputeJumpPoint(const Position& p, int dx, int dy, const Position& JumpPos);
	void ClearPreComputeJumpPoint(const Position& p);
};

///////////////////////////////////////////////////////////////////////////////
// ctor
///////////////////////////////////////////////////////////////////////////////
template <typename GRID, bool PRECOMPUTE>
PathFindRule_JPSPlusDynamic<GRID, PRECOMPUTE>::PathFindRule_JPSPlusDynamic(const GRID& g)
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
PathFindRule_JPSPlusDynamic<GRID, PRECOMPUTE>::~PathFindRule_JPSPlusDynamic()
{
	delete[] m_PreComputeTable;
}


///////////////////////////////////////////////////////////////////////////////
// 초기화(맵)
///////////////////////////////////////////////////////////////////////////////
template <typename GRID, bool PRECOMPUTE>
void PathFindRule_JPSPlusDynamic<GRID, PRECOMPUTE>::InitMap()
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
// 전처리(맵)
///////////////////////////////////////////////////////////////////////////////
template <typename GRID, bool PRECOMPUTE>
void PathFindRule_JPSPlusDynamic<GRID, PRECOMPUTE>::PrecomputeMap()
{
	bool End = false;

	// 직선 먼저 테이블을 만든다.
	for (int y = 0; y < m_Height; ++y) {
		for (int x = 0; x < m_Width; ++x) {
			if (m_Grid(x, y))
			{
				m_PreComputeTable[y * m_Width * JUMP_DIR + x * JUMP_DIR + 0] = ComputeJumpX(Position(x, y), 1, JPS::npos, End);
				m_PreComputeTable[y * m_Width * JUMP_DIR + x * JUMP_DIR + 1] = ComputeJumpX(Position(x, y), -1, JPS::npos, End);
				m_PreComputeTable[y * m_Width * JUMP_DIR + x * JUMP_DIR + 2] = ComputeJumpY(Position(x, y), 1, JPS::npos, End);
				m_PreComputeTable[y * m_Width * JUMP_DIR + x * JUMP_DIR + 3] = ComputeJumpY(Position(x, y), -1, JPS::npos, End);
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
}

template <typename GRID, bool PRECOMPUTE>
void PathFindRule_JPSPlusDynamic<GRID, PRECOMPUTE>::ChangedGrid(Position Start, Position End)
{
	if (Start.x <= End.x && Start.y <= End.y)
	{
		// 		   ###
		//  #  =>  ###
		//  	   ###

		if (Start.x > 0) {
			Start.x = Start.x - 1;
		}
		if (Start.y > 0) {
			Start.y = Start.y - 1;
		}
		if (End.x + 1 < m_Width) {
			End.x = End.x + 1;
		}
		if (End.y + 1 < m_Height) {
			End.y = End.y + 1;
		}
		
		// 가운데
		// 
		//  # 
		// 
		for (short y = Start.y + 1; y <= End.y - 1; ++y)
		{
			for (short x = Start.x + 1; x <= End.x - 1; ++x)
			{
				ClearPreComputeJumpPoint(Position(x, y));
			}
		}
	
		
		for (short y = Start.y; y <= End.y; ++y)
		{
			// 왼쪽
			// <   
			// <--# 
			// <   
			for (short x = Start.x; x >= 0; --x)
			{
				if (m_Grid(x, y) == false)
					break;

				ClearPreComputeJumpPoint(Position(x, y));
			}
		
			// 오른쪽
			//       >
			//    #-->
			//       >
			for (short x = End.x; x <= m_Width; ++x)
			{
				if (m_Grid(x, y) == false)
					break;

				ClearPreComputeJumpPoint(Position(x, y));
			}
		}

		for (short x = Start.x; x <= End.x; ++x)
		{
			// 위쪽
			//   ^^^
			//    |
			//    #

			for (short y = Start.y; y >= 0; --y)
			{
				if (m_Grid(x, y) == false)
					break;

				ClearPreComputeJumpPoint(Position(x, y));
			}
			// 아랫쪽
			//    #
			//    |
			//   VVV
			for (short y = End.y; y <= m_Height; ++y)
			{
				if (m_Grid(x, y) == false)
					break;

				ClearPreComputeJumpPoint(Position(x, y));
			}
		}
		return;
	}

	ASSERT(false);
}



///////////////////////////////////////////////////////////////////////////////
// Jump (대각선)
///////////////////////////////////////////////////////////////////////////////
template <typename GRID, bool PRECOMPUTE>
Position PathFindRule_JPSPlusDynamic<GRID, PRECOMPUTE>::JumpD(Position p, int dx, int dy, const Position& EndPos)
{
	bool ReachEndPos = false;

	p = ComputeJumpD(p, dx, dy, EndPos, ReachEndPos);

	return p;
}

///////////////////////////////////////////////////////////////////////////////
// Jump X축
///////////////////////////////////////////////////////////////////////////////
template <typename GRID, bool PRECOMPUTE>
Position PathFindRule_JPSPlusDynamic<GRID, PRECOMPUTE>::JumpX(Position p, int dx, const Position& EndPos)
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
Position PathFindRule_JPSPlusDynamic<GRID, PRECOMPUTE>::JumpY(Position p, int dy, const Position& EndPos)
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
bool PathFindRule_JPSPlusDynamic<GRID, PRECOMPUTE>::HasPreComputeJumpPoint(const Position& p, int dx, int dy) const
{
	ASSERT(!( dx == 0 && dy == 0 ));
	if (dy == 0)
	{
		if( dx > 0)
			return m_PreComputeTable[p.y * m_Width * JUMP_DIR + p.x * JUMP_DIR + 0].x != 0;
		else
			return m_PreComputeTable[p.y * m_Width * JUMP_DIR + p.x * JUMP_DIR + 1].x != 0;
	}
	else
	{
		if (dy > 0)
			return m_PreComputeTable[p.y * m_Width * JUMP_DIR + p.x * JUMP_DIR + 2].x != 0;
		else
			return m_PreComputeTable[p.y * m_Width * JUMP_DIR + p.x * JUMP_DIR + 3].x != 0;
	}
}

template <typename GRID, bool PRECOMPUTE>
Position PathFindRule_JPSPlusDynamic<GRID, PRECOMPUTE>::GetPreComputeJumpPoint(const Position& p, int dx, int dy) const
{
	ASSERT(!(dx == 0 && dy == 0));
	if (dy == 0)
	{
		if (dx > 0)
			return m_PreComputeTable[p.y * m_Width * JUMP_DIR + p.x * JUMP_DIR + 0];
		else
			return m_PreComputeTable[p.y * m_Width * JUMP_DIR + p.x * JUMP_DIR + 1];
	}
	else
	{
		if (dy > 0)
			return m_PreComputeTable[p.y * m_Width * JUMP_DIR + p.x * JUMP_DIR + 2];
		else
			return m_PreComputeTable[p.y * m_Width * JUMP_DIR + p.x * JUMP_DIR + 3];
	}
}

template <typename GRID, bool PRECOMPUTE>
void PathFindRule_JPSPlusDynamic<GRID, PRECOMPUTE>::SetPreComputeJumpPoint(const Position& p, int dx, int dy, const Position& JumpPos)
{
	ASSERT(!(dx == 0 && dy == 0));
	if (dy == 0)
	{
		if (dx > 0)
			m_PreComputeTable[p.y * m_Width * JUMP_DIR + p.x * JUMP_DIR + 0] = JumpPos;
		else
			m_PreComputeTable[p.y * m_Width * JUMP_DIR + p.x * JUMP_DIR + 1] = JumpPos;
	}
	else
	{
		if (dy > 0)
			m_PreComputeTable[p.y * m_Width * JUMP_DIR + p.x * JUMP_DIR + 2] = JumpPos;
		else
			m_PreComputeTable[p.y * m_Width * JUMP_DIR + p.x * JUMP_DIR + 3] = JumpPos;
	}
}

template <typename GRID, bool PRECOMPUTE>
void PathFindRule_JPSPlusDynamic<GRID, PRECOMPUTE>::ClearPreComputeJumpPoint(const Position& p)
{
	std::cout << "(" << p.x << "," << p.y << ")";

	m_PreComputeTable[p.y * m_Width * JUMP_DIR + p.x * JUMP_DIR + 0].x = 0;
	m_PreComputeTable[p.y * m_Width * JUMP_DIR + p.x * JUMP_DIR + 1].x = 0;
	m_PreComputeTable[p.y * m_Width * JUMP_DIR + p.x * JUMP_DIR + 2].x = 0;
	m_PreComputeTable[p.y * m_Width * JUMP_DIR + p.x * JUMP_DIR + 3].x = 0;
}


}
}

#endif
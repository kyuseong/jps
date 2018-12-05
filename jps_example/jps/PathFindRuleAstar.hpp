#ifndef PATHFIND_RULE_ASTAR_H
#define PATHFIND_RULE_ASTAR_H

namespace JPS
{
namespace Rule
{
// 길찾기 룰 (일반)
template <typename GRID>
class PathFindRule_Normal
{
	GRID m_Grid;		// 그리드(맵 정보)
public:
	PathFindRule_Normal(GRID g) : m_Grid(g) {}
	unsigned int FillNeighbors(const PathfindingNode *n, Position *wptr) const;
	Position Jump(const Position& p, const Position& src, const Position& EndPos) { return p; }
};

#define CHECKGRID(dx, dy) (m_Grid(x+(dx), y+(dy)))
#define ADDPOS(dx, dy) 	do { *w++ = Position(x+(dx), y+(dy)); } while(0)
#define ADDPOS_CHECK(dx, dy) do { if(CHECKGRID(dx, dy)) ADDPOS(dx, dy); } while(0)
#define ADDPOS_NO_TUNNEL(dx, dy) do { if(m_Grid(x+(dx),y) || m_Grid(x,y+(dy))) ADDPOS_CHECK(dx, dy); } while(0)

// Plain A* search 
template <typename GRID>
unsigned int PathFindRule_Normal<GRID>::FillNeighbors(const PathfindingNode *n, Position *wptr) const
{
	Position *w = wptr;
	const int x = n->pos.x;
	const int y = n->pos.y;
	const int d = 1;
	ADDPOS_NO_TUNNEL(-d, -d);
	ADDPOS_CHECK(0, -d);
	ADDPOS_NO_TUNNEL(+d, -d);
	ADDPOS_CHECK(-d, 0);
	ADDPOS_CHECK(+d, 0);
	ADDPOS_NO_TUNNEL(-d, +d);
	ADDPOS_CHECK(0, +d);
	ADDPOS_NO_TUNNEL(+d, +d);
	return unsigned(w - wptr);
}
#undef ADDPOS
#undef ADDPOS_CHECK
#undef ADDPOS_NO_TUNNEL
#undef CHECKGRID
}
}


#endif // PATHFINDRULE_ASTAR_H
#ifndef POSITION_H
#define POSITION_H

#include <vector>

namespace JPS
{
struct Position
{
	short x, y;

	Position() {};
	Position(short x, short y)
	{
		this->x = x;
		this->y = y;
	};

	Position(const Position& rhs)
	{
		this->x = rhs.x;
		this->y = rhs.y;
	}

	Position& operator=(const Position& rhs)
	{
		this->x = rhs.x;
		this->y = rhs.y;
		return *this;
	}

	bool operator==(const Position& p) const
	{
		return x == p.x && y == p.y;
	}
	bool operator!=(const Position& p) const
	{
		return x != p.x || y != p.y;
	}
	// for map
	bool operator<(const Position& p) const
	{
		return y < p.y || (y == p.y && x < p.x);
	}
	bool isValid() const { return x != unsigned int(-1); }
};

// for unordered_map
struct PositionHasher
{
	std::size_t operator()(const Position& k) const
	{
		return k.y << 16 | k.x;
	}
};

using PathArray = std::vector<Position>;

static const Position npos = Position(-1, -1);
}


#endif
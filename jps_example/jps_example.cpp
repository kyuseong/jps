#include "pch.h"
#include <iostream>

#include "jps/JPS.hpp"

static const char *data[] =
{
	//             1         2         3         4         
	//   01234567890123456789012345678901234567890123456789
		"##############################################",	// 0
		"#                           #                #",	// 1
		"#                   #       #        #       #",	// 2
		"#   #################       ######## #       #",	// 3
		"#                   #       #        ####### #",	// 4
		"#                   #       #        #       #",	// 5
		"################## ## #### ##########  #    #",	// 6
		"#                   #  #    #        #  #    #",	// 7
		"#                   #  #    #        #  #    #",	// 8
		"#   #################  #    #        #  #    #",	// 9
		"#                   #  #    #        #  #    #",	// 0
		"#  #    #      1    #  #2   #       #   #    #",	// 1
		"#  #    #           #  #    #      #5   #    #",	// 2
		"#  #    ########### #  #    #      #    #    #",	// 3
		"#  #    #           #  #    #      ######### #",	// 4
		"#  #    #           #  #    #      #         #",	// 5
		"#  #    #           #  #    #      #         #",	// 6
		"#  #    #           # #########  #########   #",	// 7
		"#  ##################           #   #        #",	// 8
		"#                  3#           #   #  ######",	// 9
		"#          #        #############   #        #",	// 0
		"#  ##################               #        #",	// 0
		"#                                   #    #####",	// 11
		"#          #      ###############   #        #",	// 12
		"#          #      #         4       #        #",	// 13
		"##############################################",	// 14
		nullptr
};

#define DRAW_VISITED

struct MyGrid
{
	MyGrid(const char *d[])
		: mapdata(d)
	{
		w = -1;
		h = 0;
		for (; mapdata[h]; ++h)
			w = std::min<unsigned int>(w, (unsigned int)strlen(mapdata[h]));

		out = new std::string[h];
		for (unsigned i = 0; i < h; ++i)
			out[i] = mapdata[i];
	}
	~MyGrid()
	{
		delete[] out;
	}

	bool operator()(unsigned x, unsigned y) const
	{
		if (x < w && y < h)
		{
			if (mapdata[y][x] == '#')
			{
#ifdef DRAW_VISITED 
				out[y][x] = '@';
#endif
			}
			else
			{
#ifdef DRAW_VISITED 
				if (out[y][x] == ' ')
					out[y][x] = '.';
#endif
				return true;
			}
		}

		return false;
	}

	int GetWidth() const { return w; }		/// 폭
	int GetHeight() const { return h; }		/// 높이

	unsigned w, h;
	const char **mapdata;
	std::string *out;
};

int main()
{
	MyGrid m_Grid(data);

	JPS::PathArray waypoints;
	for (char a = '1'; a <= '9'; ++a)
	{
		for (unsigned y = 0; y < m_Grid.h; ++y)
		{
			const char *sp = strchr(data[y], a);
			if (sp)
			{
				waypoints.push_back(JPS::Position(sp - data[y], y));
			}
		}
	}

	{
		JPS::PathFinder<MyGrid> search(m_Grid);

		for (unsigned i = 0; i < m_Grid.h; ++i)
			m_Grid.out[i] = m_Grid.mapdata[i];

		JPS::PathArray path;

		for (size_t i = 1; i < waypoints.size(); ++i)
		{
			path.clear();
			bool found = search.FindPath(path, JPS::Position(waypoints[1 - 1].x, waypoints[1 - 1].y), JPS::Position(waypoints[1].x, waypoints[1].y), 0);
			// bool found = search.findPath(path, JPS::Position(waypoints[i - 1].x, waypoints[i - 1].y), JPS::Position(waypoints[i].x, waypoints[i].y), 0);
			if (found)
			{
#ifdef DRAW_VISITED 
#define PUT(x, y, v) (m_Grid.out[(y)][(x)] = (v))

				unsigned c = 0;
				for (JPS::PathArray::iterator it = path.begin(); it != path.end(); ++it)
					PUT(it->x, it->y, (c++ % 26) + 'a');

				for (unsigned i = 0; i < m_Grid.h; ++i)
					std::cout << m_Grid.out[i].c_str() << std::endl;

				for (unsigned i = 0; i < m_Grid.h; ++i)
					m_Grid.out[i] = m_Grid.mapdata[i];
#endif
			}
			else
			{
				std::cout << "error";
			}

		}
	}
}

#include "PolygonPreprocessor.h"
#include "ModificationHelper.h"
#include "ModificationProcessor.h"
#include <limits>

namespace FusionCrowd {

	PolygonPreprocessor::PolygonPreprocessor(FCArray<NavMeshVetrex> & polygon)
	{
		input_poly = new Poly();
		for (auto v : polygon) {
			Vector2 vert(v.X, v.Y);
			input_poly->v.push_back(vert);
		}
	}


	PolygonPreprocessor::~PolygonPreprocessor()
	{
		delete input_poly;
	}

	float PolygonPreprocessor::performAll(ModificationProcessor& processor) {
		float res = 0;
		Poly* init_poly = new Poly();
		init_poly->v = input_poly->v;
		auto polys = decomp(init_poly);
		for (auto poly : polys) {
			float tmp = processor.CutPolygonFromMesh(poly->v);
			delete poly;
			res = tmp < res ? tmp : res;
		}
		return res;
	}

	std::vector<Poly*> PolygonPreprocessor::decomp(Poly* poly) {
		std::vector<Poly*> best1, best2, tmp1, tmp2;
		int nPoly = std::numeric_limits<int>::max();
		for (int i = 0; i < poly->v.size(); ++i) {
			if (poly->isReflex(i)) {
				for (int j = 0; j < poly->v.size(); ++j) {
					if (poly->canSee(i, j)) {
						tmp1 = decomp(poly->copy(i, j));
						tmp2 = decomp(poly->copy(j, i));
						if (tmp1.size() + tmp2.size() < nPoly) {
							clearRes(best1);
							clearRes(best2);
							best1 = tmp1;
							best2 = tmp2;
							nPoly = tmp1.size() + tmp2.size();
						}
					}
				}
			}
		}

		std::vector<Poly*> res;
		if (best1.size() != 0) {
			res.insert(res.end(), best1.begin(), best1.end());
			res.insert(res.end(), best2.begin(), best2.end());
			delete poly;
		} else {
			res.push_back(poly);
		}
		return res;
	}

	void PolygonPreprocessor::clearRes(std::vector<Poly*>& res) {
		for (auto poly : res) {
			delete poly;
		}
	}

	bool Poly::isReflex(const int &i) {
		return ModificationHelper::right(at(i - 1), at(i), at(i + 1));
	}

	bool Poly::canSee(const int &a, const int &b) {
		Vector2 p;
		float dist;

		if (ModificationHelper::leftOn(at(a + 1), at(a), at(b)) &&
			ModificationHelper::rightOn(at(a - 1), at(a), at(b))) {
			return false;
		}
		dist = Vector2::DistanceSquared(at(a), at(b));
		for (int i = 0; i < v.size(); ++i) { // for each edge
			if ((i + 1) % v.size() == a || i == a) // ignore incident edges
				continue;
			if (ModificationHelper::leftOn(at(a), at(b), at(i + 1)) &&
				ModificationHelper::rightOn(at(a), at(b), at(i))) { // if diag intersects an edge
				p = ModificationHelper::GetLineIntersectionPoint(at(a), at(b), at(i), at(i + 1));
				if (Vector2::DistanceSquared(at(a), p) < dist) { // if edge is blocking visibility to b
					return false;
				}
			}
		}

		return true;
	}

	Vector2& Poly::at(int i) {
		return v[i < 0 ? (i  + v.size()) % v.size() : i % v.size()];
	}

	Poly* Poly::copy(const int &i, const int &j) {
		Poly* res = new Poly();
		if (i < j) {
			res->v.insert(res->v.begin(), v.begin() + i, v.begin() + j + 1);
		}
		else {
			res->v.insert(res->v.begin(), v.begin() + i, v.end());
			res->v.insert(res->v.end(), v.begin(), v.begin() + j + 1);
		}
		return res;
	}
}
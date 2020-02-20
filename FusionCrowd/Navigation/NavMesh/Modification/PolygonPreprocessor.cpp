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
		ModificationHelper::SimplifyPoly(init_poly->v, false);
		auto polys = decomp(init_poly);
		for (auto poly : polys) {
			float tmp = processor.CutPolygonFromMesh(poly->v);
			delete poly;
			res = tmp < 0 ? tmp : (res + tmp);
		}
		return res;
	}

	std::vector<Poly*> PolygonPreprocessor::decomp(Poly* poly) {
		const int iter_max_num = 100;
		int v_size = poly->v.size();
		std::vector<bool> reflexed = std::vector<bool>(v_size);
		//count best possible result
		int best_poly_num = 0;
		for (int i = 0; i < poly->v.size(); ++i) {
			if (poly->isReflex(i)) {
				reflexed[i] = true;
				best_poly_num++;
			}
			else {
				reflexed[i] = false;
			}
		}
		if (best_poly_num == 0 || poly->v.size() == 3) {
			std::vector<Poly*> res;
			res.push_back(poly);
			return res;

		}
		best_poly_num /= 2;
		best_poly_num++;

		std::vector<Poly*> best1, best2, tmp1, tmp2;
		int nPoly = std::numeric_limits<int>::max();
		for (int i = 0; i < v_size; ++i) {
			if (reflexed[i]) {
				for (int j = 0; j < v_size; ++j) {
					if (poly->canSee(i, j)) {
						auto t1 = poly->copy(i, j);
						auto t2 = poly->copy(j, i);
						if (t1->v.size() >= poly->v.size() || t2->v.size() >= poly->v.size()) throw 1;
						if (t1->v.size() > t2->v.size()) {
							tmp1 = decomp(t1);
							if (tmp1.size() >= nPoly) continue;
							tmp2 = decomp(t2);
						}
						else {
							tmp2 = decomp(t2);
							if (tmp2.size() >= nPoly) continue;
							tmp1 = decomp(t1);
						}
						/*
						tmp1 = decomp(poly->copy(i, j));
						tmp2 = decomp(poly->copy(j, i));*/
						if (tmp1.size() + tmp2.size() < nPoly) {
							clearRes(best1);
							clearRes(best2);
							best1 = tmp1;
							best2 = tmp2;
							nPoly = tmp1.size() + tmp2.size();
							if (nPoly < std::numeric_limits<int>::max() && j>=iter_max_num) break;
							if (nPoly <= best_poly_num) break;
						}
					}
				}
				if (nPoly < std::numeric_limits<int>::max() && i>=iter_max_num) break;
				if (nPoly <= best_poly_num) break;
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

		if ((a + 1) % v.size() == b || (b + 1) % v.size() == a || a == b) return false; //if on same edge

		auto pointa = at(a);
		auto pointb = at(b);
		if (ModificationHelper::leftOn(at(a + 1), pointa, pointb) || //in original source was && https://mpen.ca/406/keil (polygon.cpp)
			ModificationHelper::rightOn(at(a - 1), pointa, pointb)) {
			return false;
		}
		dist = Vector2::DistanceSquared(pointa, pointb);
		for (int i = 0; i < v.size(); ++i) { // for each edge
			if ((i + 1) % v.size() == a || i == a) // ignore incident edges
				continue;
			if (ModificationHelper::leftOn(pointa, pointb, at(i + 1)) &&
				ModificationHelper::rightOn(pointa, pointb, at(i))) { // if diag intersects an edge
				p = ModificationHelper::GetLineIntersectionPoint(pointa, pointb, at(i), at(i + 1));
				if (Vector2::DistanceSquared(pointa, p) < dist) { // if edge is blocking visibility to b
					return false;
				}
			}
		}

		return true;
	}

	Vector2& Poly::at(int i) {
		return v.at((i + v.size()) % v.size());
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
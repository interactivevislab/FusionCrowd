#include "ModificationHelper.h"
#include <algorithm>

namespace FusionCrowd
{
	/*Returns is point under line v0->v1 (or point.x< line.x if line is vertical)*/
	bool ModificationHelper::IsPointUnderLine(Vector2 v0, Vector2 v1, Vector2 point, bool reverse, bool strict) {
		bool vertical = false;
		bool res;
		float k = 0.0;
		float c = 0.0;
		if (v0.x == v1.x) {
			vertical = true;
		}
		else {
			k = (v0.y - v1.y) / (v0.x - v1.x);
			c = v0.y - k * v0.x;
		}
		if (vertical) {
			if (strict) {
				res = point.x < v0.x;
			}
			else {
				res = point.x <= v0.x;
			}
		}
		else {
			if (strict) {
				res = point.y < k*point.x + c;
			}
			else {
				res = point.y <= k * point.x + c;
			}
		}

		return reverse ? !res : res;
	}

	bool ModificationHelper::IsClockwise(FCArray<NavMeshVetrex> & polygon) {
		float sum = 0;
		for (int i = 0; i < polygon.size(); i++) {
			NavMeshVetrex v0 = polygon[i];
			NavMeshVetrex v1 = polygon[(i + 1) % polygon.size()];
			sum += (v1.X - v0.X)*(v0.Y + v1.Y);
		}

		return sum > 0;
	}

	std::vector<Vector2> ModificationHelper::FindPolyAndSegmentCrosspoints(Vector2 q, Vector2 v1, NavMeshPoly* poly, bool ray_mode) {
		//https://stackoverflow.com/questions/563198/how-do-you-detect-where-two-line-segments-intersect
		std::vector<Vector2> res = std::vector<Vector2>();
		//u = (q-p)Xr/(rXs)
		//vXw = v.x*w.y-v.y*w.x
		auto s = v1 - q;
		for (int i = 0; i < poly->vertCount; i++) {
			auto p = poly->vertices[poly->vertIDs[i]];
			auto r = poly->vertices[poly->vertIDs[(i + 1) % poly->vertCount]] - p;

			auto rXs = r.x*s.y - r.y*s.x;
			if (fabs(rXs)<1e-7f) continue;
			auto delta = q - p;
			auto deltaXr = delta.x*r.y - delta.y*r.x;
			float u = deltaXr / rXs;
			float deltaXs = delta.x*s.y - delta.y*s.x;
			float t = deltaXs / rXs;
			if (!ray_mode && u > 0.0f && u < 1.0f && t < 1.0f && t > 0.0f) {
					res.push_back(q + u * s);
			}
			if (ray_mode && u > 0.99f  && t <= 1.0f && t >= 0.0f) {
				res.push_back(q + u * s);
			}
		}
		return res;
	}

	/*Sort poly vertices to make correct poly (with no segment intersections*/
	void ModificationHelper::ResetNodePolySequence(NavMeshNode& node) {
		Vector2 last_added;
		std::vector<size_t> ids_left = std::vector<size_t>(node._poly.vertCount);
		std::vector<size_t> res = std::vector<size_t>(node._poly.vertCount);

		Vector2 mean = Vector2(0, 0);
		for (int i = 0; i < node._poly.vertCount; i++) {
			ids_left[i] = node._poly.vertIDs[i];
			mean += node._poly.vertices[node._poly.vertIDs[i]];
		}
		mean /= node._poly.vertCount;

		Vector2 prev_dir = node._poly.vertices[ids_left[0]] - mean;
		prev_dir.Normalize();
		size_t added = 0;
		while (ids_left.size() > 0) {
			float max_product = -FLT_MAX;
			int candidateidpos = 0;
			for (int i = 0; i < ids_left.size(); i++) {
				Vector2 mean_v_vec = node._poly.vertices[ids_left[i]] - mean;
				mean_v_vec.Normalize();
				float dotpr = prev_dir.Dot(mean_v_vec);
				float dir = mean_v_vec.x * prev_dir.y - mean_v_vec.y * prev_dir.x;
				if (dotpr > max_product && dir > 0) {
					max_product = dotpr;
					candidateidpos = i;
				}
			}
			res[added] = ids_left[candidateidpos];
			prev_dir = node._poly.vertices[ids_left[candidateidpos]] - mean;
			prev_dir.Normalize();
			added++;
			ids_left.erase(ids_left.begin() + candidateidpos);
		}

		//fill res
		for (int i = 0; i < res.size(); i++) {
			node._poly.vertIDs[i] = res[i];
		}
	}

	/*make hull concave*/
#pragma region ConcaveHullHelpFunctions
	bool cmp(Vector2 a, Vector2 b) {
		return a.x < b.x || a.x == b.x && a.y < b.y;
	}

	bool cw(Vector2 a, Vector2 b, Vector2 c) {
		return a.x*(b.y - c.y) + b.x*(c.y - a.y) + c.x*(a.y - b.y) < 0;
	}

	bool ccw(Vector2 a, Vector2 b, Vector2 c) {
		return a.x*(b.y - c.y) + b.x*(c.y - a.y) + c.x*(a.y - b.y) > 0;
	}
#pragma endregion

	void ModificationHelper::ConcaveHull(std::vector<Vector2>& poly) {
		if (poly.size() == 1)  return;
		std::sort(poly.begin(), poly.end(), &cmp);
		Vector2 p1 = poly[0], p2 = poly.back();
		std::vector<Vector2> up, down;
		up.push_back(p1);
		down.push_back(p1);
		for (size_t i = 1; i < poly.size(); ++i) {
			if (i == poly.size() - 1 || cw(p1, poly[i], p2)) {
				while (up.size() >= 2 && !cw(up[up.size() - 2], up[up.size() - 1], poly[i]))
					up.pop_back();
				up.push_back(poly[i]);
			}
			if (i == poly.size() - 1 || ccw(p1, poly[i], p2)) {
				while (down.size() >= 2 && !ccw(down[down.size() - 2], down[down.size() - 1], poly[i]))
					down.pop_back();
				down.push_back(poly[i]);
			}
		}
		poly.clear();
		for (size_t i = 0; i < up.size(); ++i)
			poly.push_back(up[i]);
		for (size_t i = down.size() - 2; i > 0; --i)
			poly.push_back(down[i]);
	}

	void ModificationHelper::SimplifyPoly(std::vector<Vector2> &poly) {
		ModificationHelper::ConcaveHull(poly);
		//remove close points
		std::vector<float> distances = std::vector<float>(poly.size());
		for (int i = 0; i < poly.size(); i++) {
			distances[i] = Vector2::Distance(poly[i], poly[(i + 1) % poly.size()]);
		}
		for (int i = poly.size() - 1; i >= 0; i--) {
			if (distances[i] < 0.5f) {
				poly.erase(poly.begin() + i);
			}
		}
		//remove smooth lines
		std::vector<bool> delete_mark = std::vector<bool>(poly.size());
		for (int i = 0; i < poly.size(); i++) {
			Vector2 v0 = poly[(i + 1) % poly.size()] - poly[i];
			Vector2 v1 = poly[(i + 2) % poly.size()] - poly[(i + 1) % poly.size()];
			v0.Normalize();
			v1.Normalize();
			if (v0.Dot(v1) > 0.996f) {
				delete_mark[(i + 1) % poly.size()] = true;
			}
			else {
				delete_mark[(i + 1) % poly.size()] = false;

			}

		}
		for (int i = poly.size() - 1; i >= 0; i--) {
			if (delete_mark[i]) {
				poly.erase(poly.begin() + i);
			}
		}
	}

	bool ModificationHelper::ValidateModificator(NodeModificator * modificator) {
		if (modificator->modification_type != CUT_CURVE) return true;
		auto& poly = modificator->polygon_to_cut;
		bool f = true;
		while (f) {
			f = false;
			for (int i = poly.size() - 1; i >= 0; i--) {
				if ((poly[i] - poly[(i + poly.size() - 1) % poly.size()]).Length() < 1e-3f) {
					poly.erase(poly.begin() + i);
					modificator->polygon_vertex_ids.erase(modificator->polygon_vertex_ids.begin() + i);
					f = true;
					break;
				}
			}
		}
		auto& node_poly = modificator->node->_poly;
		//remove vertexes on edge
		//todo
		for (int i = 1; i < poly.size() - 1; i++) {
			auto v0 = poly[i];
			for (int j = 0; j < node_poly.vertCount; j++) {
				auto v1 = node_poly.vertices[node_poly.vertIDs[j]];
				auto v2 = node_poly.vertices[node_poly.vertIDs[(j + 1) % node_poly.vertCount]];
				if (fabs(v2.x - v1.x) < 1e-6f) {
					//if v1-v2 - vertical
					if (fabs(v0.x - v2.x) < 1e-6f) return false;
				}
				else {
					float k0 = (v2.y - v1.y) / (v2.x - v1.x);
					float c0 = v2.y - k0 * v2.x;
					if (fabs(k0*v0.x + c0 - v0.y) < 1e-6) return false;
				}
			}

		}
		return poly.size() > 2;
	}
}

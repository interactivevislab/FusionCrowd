#include "ModificationHelper.h"
#include <algorithm>
#include <set>

using namespace DirectX::SimpleMath;

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

	bool ModificationHelper::IsClockwise(std::vector<Vector2>& polygon) {
		float sum = 0;
		for (int i = 0; i < polygon.size(); i++) {
			Vector2 v0 = polygon[i];
			Vector2 v1 = polygon[(i + 1) % polygon.size()];
			sum += (v1.x - v0.x)*(v0.y + v1.y);
		}

		return sum > 0;
	}

	std::vector<Vector2> ModificationHelper::FindPolyAndSegmentCrosspoints(Vector2 q, Vector2 v1, NavMeshPoly* poly, bool ray_mode)
	{
		//https://stackoverflow.com/questions/563198/how-do-you-detect-where-two-line-segments-intersect
		std::vector<Vector2> res = std::vector<Vector2>();

		auto addToResultSkipDuplicates = [&] (Vector2 v) {
			for(auto r : res)
			{
				if(Vector2::Distance(r, v) < 1e-4)
					return;
			}

			res.push_back(v);
		};


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
			if (!ray_mode && u >= 0.0f && u <= 1.0f && t <= 1.0f && t >= 0.0f) {
				addToResultSkipDuplicates(q + u * s);
			}
			if (ray_mode && u > 0.99f && t <= 1.0f && t >= 0.0f) {
				addToResultSkipDuplicates(q + u * s);
			}
		}

		return res;
	}

	void ModificationHelper::RemoveDuplicateVerticesFromNodePoly(NavMeshNode& node) {
		std::vector<Vector2> vertices;
		std::vector<unsigned int> ids;
		for (int i = 0; i < node._poly.vertCount; i++) {
			vertices.push_back(node._poly.getVertexByPos(i));
			ids.push_back(node._poly.vertIDs[i]);
		}
		for (int i = vertices.size() - 1; i >= 0; i--) {
			for (int j = vertices.size() - 1; j >= 0; j--) {
				if (i == j) continue;
				if (Vector2::Distance(vertices[i], vertices[j]) < 1e-6f) {
					vertices.erase(vertices.begin() + i);
					ids.erase(ids.begin() + i);
					break;
				}
			}
		}
		node._poly.vertCount = ids.size();
		delete[] node._poly.vertIDs;
		node._poly.vertIDs = new unsigned int[ids.size()];
		for (int i = 0; i < ids.size(); i++) {
			node._poly.vertIDs[i] = ids[i];
		}
	}

	/*Sort cut_poly vertices to make correct cut_poly (with no segment intersections*/
	void ModificationHelper::ResetNodePolySequence(NavMeshNode& node) {
		Vector2 last_added;
		std::vector<size_t> ids_left = std::vector<size_t>(node._poly.vertCount);
		std::vector<size_t> res = std::vector<size_t>(node._poly.vertCount);

		Vector2 mean = Vector2(0, 0);
		for (int i = 0; i < node._poly.vertCount; i++) {
			ids_left[i] = node._poly.vertIDs[i];
			Vector2 v = node._poly.vertices[node._poly.vertIDs[i]];
			mean += v;
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
		Vector2 v;
		for (int i = 0; i < node._poly.vertCount; i++) {
			v = node._poly.vertices[node._poly.vertIDs[i]];
		}
	}

	/*make hull concave*/
#pragma region ConcaveHullHelpFunctions
	namespace {
		bool cmp(Vector2 a, Vector2 b) {
			return a.x < b.x || a.x == b.x && a.y < b.y;
		}

		bool cw(Vector2 a, Vector2 b, Vector2 c) {
			return a.x*(b.y - c.y) + b.x*(c.y - a.y) + c.x*(a.y - b.y) < 0;
		}

		bool ccw(Vector2 a, Vector2 b, Vector2 c) {
			return a.x*(b.y - c.y) + b.x*(c.y - a.y) + c.x*(a.y - b.y) > 0;
		}
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

	void ModificationHelper::SimplifyPoly(std::vector<Vector2> &poly, bool makeConcave) {
		if (makeConcave) ModificationHelper::ConcaveHull(poly);
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

	bool IsSetsIntersects(std::set<int>& s0, std::set<int>& s1) {
		for (auto e : s0) {
			if (s1.count(e) > 0) return true;
		}
		return false;
	}

	bool ModificationHelper::ValidateModificator(NodeModificator * modificator, std::vector<NodeModificator*>& modifications) {
		if (modificator->modification_type != CUT_CURVE) return true;
		auto& cut_poly = modificator->polygon_to_cut;
		bool f = true;
		//remove point which to close to each other
		while (f) {
			f = false;
			for (int i = cut_poly.size() - 1; i >= 0; i--) {
				if ((cut_poly[i] - cut_poly[(i + cut_poly.size() - 1) % cut_poly.size()]).Length() < 1e-3f) {
					cut_poly.erase(cut_poly.begin() + i);
					modificator->polygon_vertex_ids.erase(modificator->polygon_vertex_ids.begin() + i);
					f = true;
					break;
				}
			}
		}
		if (cut_poly.size() < 2) return false;
		auto& node_poly = modificator->node->_poly;

		//add new mods
		std::map<int, std::set<int>> vert2edges;
		bool modyfied = false;
		for (int i = 0; i < cut_poly.size(); i++) {
			auto v0 = cut_poly[i];
			vert2edges.insert({ i, std::set<int>() });
			for (int j = 0; j < node_poly.vertCount; j++) {
				auto v1 = node_poly.vertices[node_poly.vertIDs[j]];
				auto v2 = node_poly.vertices[node_poly.vertIDs[(j + 1) % node_poly.vertCount]];
				if (IsPointsOnLine(v0, v1, v2)) vert2edges[i].insert(j);
			}
			if (i > 0 && i < cut_poly.size() - 1 && vert2edges[i].size() > 0) modyfied = true;
		}
		modyfied = modyfied || cut_poly.size() == 2;
		if (!modyfied) return cut_poly.size() > 2;
		NodeModificator* cur_mod = nullptr;
		if (vert2edges[0].size() == 0) return false;
		for (int i = 0; i < cut_poly.size()-1; i++) {
			if (vert2edges[i].size() > 0) {
				if (cur_mod != nullptr &&
					(i == 0 || !IsSetsIntersects(vert2edges[i - 1], vert2edges[i]))) {
					cur_mod->polygon_to_cut.push_back(modificator->polygon_to_cut[i]);
					cur_mod->polygon_vertex_ids.push_back(modificator->polygon_vertex_ids[i]);
					if (cur_mod->polygon_to_cut.size() == 2) cur_mod->modification_type = SPLIT;
					if (cur_mod->polygon_to_cut.size() > 1) {
						modifications.push_back(cur_mod);
					}
					else delete cur_mod;
				}
				cur_mod = new NodeModificator();
				cur_mod->correct = true;
				cur_mod->node = modificator->node;
				cur_mod->side = IsPointUnderLine(cut_poly[i], cut_poly[(i + 1) % cut_poly.size()], cut_poly[(i + 2) % cut_poly.size()], true, true);
				cur_mod->modification_type = CUT_CURVE;
				cur_mod->polygon_to_cut.push_back(modificator->polygon_to_cut[i]);
				cur_mod->polygon_vertex_ids.push_back(modificator->polygon_vertex_ids[i]);
			}
			if (vert2edges[i].size() == 0) {
				cur_mod->polygon_to_cut.push_back(modificator->polygon_to_cut[i]);
				cur_mod->polygon_vertex_ids.push_back(modificator->polygon_vertex_ids[i]);
			}
		}
		if (cur_mod != nullptr &&
			!IsSetsIntersects(vert2edges[vert2edges.size() - 2], vert2edges[vert2edges.size() - 1])) {
			cur_mod->polygon_to_cut.push_back(modificator->polygon_to_cut[cut_poly.size() - 1]);
			cur_mod->polygon_vertex_ids.push_back(modificator->polygon_vertex_ids[cut_poly.size() - 1]);
			if (cur_mod->polygon_to_cut.size() == 2) cur_mod->modification_type = SPLIT;
			modifications.push_back(cur_mod);
		}
		else delete cur_mod;
		return false;
	}

	bool ModificationHelper::IsPointsOnLine(Vector2 v0, Vector2 v1, Vector2 v2, float delta) {
		return fabs((v0.y - v1.y) * (v0.x - v2.x) - (v0.y - v2.y) * (v0.x - v1.x)) < delta;
	}

	float ModificationHelper::area(Vector2 &a,  Vector2 &b,  Vector2 &c) {
		return (((b.x - a.x)*(c.y - a.y)) - ((c.x - a.x)*(b.y - a.y)));
	}

	bool ModificationHelper::right(Vector2 &a, Vector2 &b, Vector2 &c) {
		return area(a, b, c) < 0;
	}

	bool ModificationHelper::leftOn(Vector2 &a, Vector2 &b, Vector2 &c) {
		return area(a, b, c) >= 0;
	}

	bool ModificationHelper::rightOn(Vector2 &a, Vector2 &b, Vector2 &c) {
		return area(a, b, c) <= 0;
	}

	Vector2 ModificationHelper::GetLineIntersectionPoint(Vector2& p0, Vector2& p1, Vector2& o1, Vector2& o2) {
		Vector2 res;
		float a1, b1, c1, a2, b2, c2, det;
		a1 = p1.y - p0.y;
		b1 = p0.x - p1.x;
		c1 = a1 * p0.x + b1 * p0.y;
		a2 = o2.y - o1.y;
		b2 = o1.x - o2.x;
		c2 = a2 * o1.x + b2 * o1.y;
		det = a1 * b2 - a2 * b1;
		if (!abs(det)<1e-8f) { // lines are not parallel
			res.x = (b2 * c1 - b1 * c2) / det;
			res.y = (a1 * c2 - a2 * c1) / det;
		}
		return res;
	}
}

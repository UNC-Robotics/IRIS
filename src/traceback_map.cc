#include "traceback_map.h"
#include <assert.h> 

TracebackMap::TracebackMap(const Idx num_vertices) : n_(num_vertices) {
	Reset();
}

TracebackMap::~TracebackMap() {
	map_.resize(boost::extents[0][0]);
	cost_array_.resize(0);
	edge_cost_array_.resize(0);
	neighbors_.resize(0);
}

void TracebackMap::Reset() {
	if (map_locked_) {
		std::cout << "Map is locked!" << std::endl;
		std::cout << __func__ << " is called!" << std::endl;
		getchar();
	}
	map_.resize(boost::extents[n_][n_]);
	cost_array_.resize( (n_*(n_-1))/2, R_INF );
	edge_cost_array_.resize( (n_*(n_-1))/2, R_INF );
	neighbors_.resize(n_);

	for (Idx i = 0; i < n_; ++i) {
		for (Idx j = 0; j < n_; ++j) {
			map_[i][j] = I_INF;
		}
		neighbors_[i].clear();
	}
}

SizeType TracebackMap::FlatIndex(const SizeType i, const SizeType j) const {
	if (i < j) {
		return n_*i + j - 1 - (i+3)*i/2;
	}
	else if (j < i) {
		return n_*j + i - 1 - (j+3)*j/2;
	}

	std::cerr << "Invalid query (i = j)!" << std::endl;
	exit(1);
}

void TracebackMap::AddTrace(const Idx i, const Idx j, const Idx i_to_j) {
	if (map_locked_) {
		std::cout << "Map is locked!" << std::endl;
		std::cout << __func__ << " is called!" << std::endl;
		getchar();
	}
	map_[i][j] = i_to_j;
}

void TracebackMap::AddCost(const Idx i, const Idx j, const RealNum cost) {
	if (map_locked_) {
		std::cout << "Map is locked!" << std::endl;
		std::cout << __func__ << " is called!" << std::endl;
		getchar();
	}
	cost_array_[FlatIndex(i,j)] = cost;
}

void TracebackMap::AddEdgeCost(const Idx i, const Idx j, const RealNum cost) {
	if (map_locked_) {
		std::cout << "Map is locked!" << std::endl;
		std::cout << __func__ << " is called!" << std::endl;
		getchar();
	}
	edge_cost_array_[FlatIndex(i,j)] = cost;
}

void TracebackMap::AddNeighbor(const Idx i, const Idx j) {
	if (map_locked_) {
		std::cout << "Map is locked!" << std::endl;
		std::cout << __func__ << " is called!" << std::endl;
		getchar();
	}
	neighbors_[i].insert(j);
	neighbors_[j].insert(i);
}

void TracebackMap::AddDirectEdge(const Idx i, const Idx j, const RealNum cost) {
	if (map_locked_) {
		std::cout << "Map is locked!" << std::endl;
		std::cout << __func__ << " is called!" << std::endl;
		getchar();
	}
	AddTrace(i, j, j);
	AddTrace(j, i, i);
	AddEdgeCost(i, j, cost);
	AddNeighbor(i, j);
}

void TracebackMap::RemoveDirectEdge(const Idx i, const Idx j) {
	if (map_locked_) {
		std::cout << "Map is locked!" << std::endl;
		std::cout << __func__ << " is called!" << std::endl;
		getchar();
	}
	// currently there's no need to deal with trace and cost
	// considering just use a list of dist and a list of last steps
	neighbors_[i].erase(j);
	neighbors_[j].erase(i);
	edge_cost_array_[FlatIndex(i,j)] = R_INF;
}

Idx TracebackMap::Trace(const Idx i, const Idx j) const {
	if (map_[i][j] == I_INF) {
		std::cout << i << " to " << j << " trace does not exist" << std::endl;
		exit(1);
	}

	return map_[i][j];
}

RealNum TracebackMap::Cost(const Idx i, const Idx j) const {
	return cost_array_[FlatIndex(i,j)];
}

RealNum TracebackMap::EdgeCost(const Idx i, const Idx j) const {
	auto cost = edge_cost_array_[FlatIndex(i,j)];
	if (cost < R_INF) {
		return cost;
	}

	std::cout << i << " to " << j << " edge does not exist" << std::endl;
	exit(1);
}

std::set<Idx> TracebackMap::NeighborList(const Idx i) const {
	return neighbors_[i];
}

std::vector<std::vector<Idx>> TracebackMap::Successors(const Idx source, const VisibilitySet& current_vis_set, const Inspection::GPtr graph) {
	if (map_locked_) {
		std::cout << "Map is locked!" << std::endl;
		std::cout << __func__ << " is called!" << std::endl;
		getchar();
	}
	std::vector<std::vector<Idx>> successors;

	std::vector<Idx> last_step(n_, n_);
	std::vector<RealNum> dist(n_, R_INF);
	DijkstraQueue queue;
	dist[source] = 0;
	last_step[source] = source;
	queue.push({source, 0});

	while (!queue.empty()) {
		auto s = queue.top();
		queue.pop();
		Idx u = s.first;

		if (s.second > dist[u]) {
			continue;
		}

		if (!current_vis_set.Contains(graph->Vertex(u)->vis)) {
			std::vector<Idx> local_path;
			Idx tag = u;
			local_path.push_back(u);
			while (tag != source) {
				AddTrace(tag, source, last_step[tag]);
				tag = last_step[tag];
				local_path.push_back(tag);
			}
			successors.push_back(local_path);
			// AddCost(u, source, dist[u]);
			
			continue;
		}

		auto neig = NeighborList(u);
		for (const auto &v : neig) {
			auto edge_cost = EdgeCost(u, v);
			assert(edge_cost > 0);
			if (dist[v] > dist[u] + edge_cost) {
				dist[v] = dist[u] + edge_cost;
				last_step[v] = u;
				queue.push({v, dist[v]});

				if (dist[v] < Cost(v, source)) {
					AddTrace(v, source, u);
					AddCost(v, source, dist[v]);
				}
			}
		}
	}

	return successors;
}

std::vector<RealNum> TracebackMap::SingleSourceShortestDistance(const Idx source, bool query, const Idx target) const {
	std::vector<Idx> last_step(n_, n_);
	std::vector<RealNum> dist(n_, R_INF);
	DijkstraQueue queue;
	dist[source] = 0;
	last_step[source] = source;
	queue.push({source, 0});

	while (!queue.empty()) {
		auto s = queue.top();
		queue.pop();
		Idx u = s.first;

		auto neig = NeighborList(u);
		for (const auto &v : neig) {
			auto edge_cost = EdgeCost(u, v);
			assert(edge_cost > 0);
			if (dist[v] > dist[u] + edge_cost) {
				dist[v] = dist[u] + edge_cost;
				last_step[v] = u;
				queue.push({v, dist[v]});
			}
		}
	}

	if (query) {
		auto tag = target;
		std::cout << "Queried path: ";
		while (tag != source) {
			std::cout << tag << ", ";
			tag = last_step[tag];
		}
		std::cout << std::endl;
	}

	return dist;
}

std::vector<std::vector<Idx>> TracebackMap::FirstMeetSuccessors(const Idx source, const VisibilitySet& current_vis_set, const Inspection::GPtr graph) {
	if (map_locked_) {
		std::cout << "Map is locked!" << std::endl;
		std::cout << __func__ << " is called!" << std::endl;
		getchar();
	}
	std::vector<std::vector<Idx>> successors;
	std::vector<Idx> last_step(n_, n_);
	std::vector<RealNum> dist(n_, R_INF);
	DijkstraQueueNew queue;
	dist[source] = 0;
	last_step[source] = source;
	queue.push(std::make_tuple(source, 0, true));
	SizeType valid_num = 1;

	while (valid_num > 0) {
		auto s = queue.top();
		queue.pop();

		Idx u = std::get<0>(s);
		bool valid_thread = std::get<2>(s);
		if (valid_thread) {
			valid_num--;
		}

		if (std::get<1>(s) > dist[u]) {
			continue;
		}

		if (valid_thread && !current_vis_set.Contains(graph->Vertex(u)->vis)) {
			std::vector<Idx> local_path;
			Idx tag = u;
			local_path.push_back(u);
			while (tag != source) {
				AddTrace(tag, source, last_step[tag]);
				tag = last_step[tag];
				local_path.push_back(tag);
			}
			successors.push_back(local_path);
			// AddCost(u, source, dist[u]);
			valid_thread = false;
		}

		auto neig = NeighborList(u);
		for (const auto &v : neig) {
			auto edge_cost = EdgeCost(u, v);
			assert(edge_cost > 0);
			if (dist[v] > dist[u] + edge_cost) {
				dist[v] = dist[u] + edge_cost;
				last_step[v] = u;
				queue.push(std::make_tuple(v, dist[v], valid_thread));
				if (valid_thread) {
					valid_num++;
				}

				if (dist[v] < Cost(v, source)) {
					AddTrace(v, source, u);
					AddCost(v, source, dist[v]);
				}
			}
		}
	}

	return successors;
}

std::vector<std::vector<Idx>> TracebackMap::NeighboringSuccessors(const Idx source) {
	auto neig = NeighborList(source);
	std::vector<std::vector<Idx>> successors;

	for (const auto& v : neig) {
		std::vector<Idx> local_path{v, source};
		successors.push_back(local_path);
		AddCost(v, source, EdgeCost(v, source));
	}

	return successors;
}

std::list<Idx> TracebackMap::FullPath(const std::vector<Idx> milestones) const {
	std::list<Idx> path;
	SizeType len = milestones.size();

	for (Idx i = 0; i < len-1; ++i) {
		Idx t = milestones[i];
		Idx s = milestones[i+1];

		while (t != s) {
			path.push_front(t);
			t = Trace(t, s);
		}
	}

	path.push_front(milestones[len-1]);

	return path;
}

bool TracebackMap::Connected(const Inspection::GPtr& graph) const {
	if (n_ < 2) {
		return true;
	}
	
	for (Idx i = 0; i < n_; ++i) {
		auto neig = neighbors_[i];
		bool valid_found = false;
		for (auto j : neig) {
			auto e = graph->FindEdge(i, j);
			if (e->valid) {
				valid_found = true;
				break;
			}
		}

		if (!valid_found) {
			std::cout << i << std::endl;
			return false;
		}
	}

	return true;
}

#if USE_HEURISTIC
RealNum TracebackMap::ComputeHeuristic(const Idx source, const VisibilitySet& current_vis_set, const Inspection::GPtr graph, const VisibilitySet& graph_coverage) {
	std::vector<Idx> last_step(n_, n_);
	std::vector<RealNum> dist(n_, R_INF);
	DijkstraQueue queue;
	dist[source] = 0;
	last_step[source] = source;
	queue.push({source, 0});
	VisibilitySet coverage = current_vis_set;
	auto max_size = HEUR_PORTION * (graph_coverage.Size() - current_vis_set.Size()) + current_vis_set.Size();

	while (!queue.empty()) {
		auto s = queue.top();
		queue.pop();
		Idx u = s.first;

		if (s.second > dist[u]) {
			continue;
		}

		coverage.Insert(graph->Vertex(u)->vis);

		if (coverage.Size() >= max_size) {
			return dist[u];
		}

		auto neig = NeighborList(u);
		for (const auto &v : neig) {
			auto edge_cost = EdgeCost(u, v);
			assert(edge_cost > 0);
			if (dist[v] > dist[u] + edge_cost) {
				dist[v] = dist[u] + edge_cost;
				last_step[v] = u;
				queue.push({v, dist[v]});
			}
		}
	}

	std::cerr << "Heuristic computation failed!" << std::endl;
	return 0;
}
#endif

void TracebackMap::CheckLowerBound(const Idx source, const String message) const {
	auto d = SingleSourceShortestDistance(source);
	for (auto i = 0; i < n_; ++i) {
		if (i == source) {
			continue;
		}

		if (Cost(i, source) < d[i]) {
			std::cout << message << std::endl;
			std::cout << "Checking lower bound!" << std::endl;
			std::cout << source << " => " << i << std::endl;
			std::cout << "Dijkstra d: " << d[i] << std::endl;
			std::cout << "Map cost: " << Cost(i, source) << std::endl;
			getchar();

			auto d_inv = SingleSourceShortestDistance(i);
			std::cout << "Inverse Dijkstra d: " << d_inv[source] << std::endl;
			getchar();

			auto q = SingleSourceShortestDistance(source, true, i);
			getchar();
		}
	}
}

void TracebackMap::CheckAllLowerBound(const String message) const {
	for (auto i = 0; i < n_; ++i) {
		CheckLowerBound(i, message);
	}
}
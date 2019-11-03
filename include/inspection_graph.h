#ifndef INSPECTION_GRAPH_H_
#define INSPECTION_GRAPH_H_

#include <set>
#include <fstream>

#include <ompl/base/spaces/RealVectorStateSpace.h>

#include "visibility_set.h"
#include "ompl/crisp_state_space.h"
#include "ompl/drone_state_space.h"

namespace Inspection {
	class Vertex {
	public:
		Vertex(const Idx i);

		const Idx index;
		ompl::base::State *state{nullptr};
		SizeType time_vis{0};
		SizeType time_build{0};
		VisibilitySet vis;
		// std::set<Idx> outgoing_edges;
	};

	class Edge {
	public:
		Edge(const Idx s, const Idx t);

		const Idx source;
		const Idx target;
		bool checked{false}; // only tree edges are marked checked
		bool valid{false};
		SizeType time_forward_kinematics{0};
		SizeType time_collision_detection{0};
		RealNum cost;

		bool in_virtual_graph{false};
		bool virtual_checked{false};
	};

	using VPtr = std::shared_ptr<Vertex>;
	using EPtr = std::shared_ptr<Edge>;

	class Graph {
	public:
		Graph();
		~Graph();

		void AddVertex(const Idx i);
		void AddVertex(VPtr vertex);
		void AddEdge(const Idx s, const Idx t);
		void AddEdge(EPtr edge);
		EPtr FindEdge(const Idx s, const Idx t) const;
		void UpdateGlobalVisibility(const VisibilitySet& set);

		Idx NumVertices() const;
		Idx NumEdges() const;

		VPtr& Vertex(const Idx index);
		VPtr Vertex(const Idx index) const;
		EPtr& Edge(const Idx index);
		EPtr Edge(const Idx index) const;
        SizeType NumTargetsCovered() const;

		void Save(const String file_name, const bool save_configs=true, const Idx dof=0) const;
		void ReadFromFiles(const String file_name, const bool read_configs=true, const Idx dof=0);

	private:
		void Reset();
		VPtr null_vertex_{nullptr};
		EPtr null_edge_{nullptr};
		std::vector<VPtr> vertices_;
		std::vector<EPtr> edges_;
		VisibilitySet global_vis_set_;

	}; // Graph

	using GPtr = std::shared_ptr<Graph>;
}

#endif // INSPECTION_GRAPH_H_

#include "inspection_graph.h"

Inspection::Graph::Graph() {
	Reset();
}

Inspection::Graph::~Graph() {
	Reset();
}

void Inspection::Graph::Reset() {
	vertices_.resize(0);
	edges_.resize(0);
	global_vis_set_.Clear();
}

Inspection::Vertex::Vertex(const Idx i) : index(i) {

}

Inspection::Edge::Edge(const Idx s, const Idx t) : source(s), target(t) {

}

void Inspection::Graph::AddVertex(const Idx i) {
	if (i != vertices_.size()) {
		std::cerr << "Incorrect index for new vertex to add!" << std::endl;
		exit(1);
	}

	VPtr v(new Inspection::Vertex(i));
	vertices_.push_back(v);
}

void Inspection::Graph::AddVertex(VPtr vertex) {
	if (vertex->index != vertices_.size()) {
		std::cerr << "Incorrect index for new vertex to add!" << std::endl;
		exit(1);
	}

	vertices_.push_back(vertex);
}

void Inspection::Graph::AddEdge(const Idx s, const Idx t) {
	if (FindEdge(s, t)) {
		std::cerr << "Edge should not duplicate!" << std::endl;
		exit(1);
	}

	EPtr e(new Inspection::Edge(s, t));
	edges_.push_back(e);
}

void Inspection::Graph::AddEdge(EPtr edge) {
	if (FindEdge(edge->source, edge->target)) {
		std::cerr << "Edge should not duplicate!" << std::endl;
		exit(1);
	}

	edges_.push_back(edge);
}

Inspection::EPtr Inspection::Graph::FindEdge(const Idx s, const Idx t) const{
	for (auto && e : edges_) {
		if (e->source == s && e->target == t) {
			return e;
		}

		if (e->source == t && e->target == s) {
			return e;
		}
	}

	return nullptr;
}

void Inspection::Graph::UpdateGlobalVisibility(const VisibilitySet& set) {
	global_vis_set_.Insert(set);
}

Idx Inspection::Graph::NumVertices() const {
	return vertices_.size();
}
	
Idx Inspection::Graph::NumEdges() const {
	return edges_.size();
}

Inspection::VPtr& Inspection::Graph::Vertex(const Idx index) {
	if (index < NumVertices()){
		return vertices_[index];
	}

	return null_vertex_;
}
	
Inspection::VPtr Inspection::Graph::Vertex(const Idx index) const {
	if (index < NumVertices()){
		return vertices_[index];
	}

	return null_vertex_;
}
	
Inspection::EPtr& Inspection::Graph::Edge(const Idx index) {
	if (index < NumEdges()){
		return edges_[index];
	}

	return null_edge_;
}

Inspection::EPtr Inspection::Graph::Edge(const Idx index) const{
	if (index < NumEdges()){
		return edges_[index];
	}

	return null_edge_;
}

SizeType Inspection::Graph::NumTargetsCovered() const {
    return global_vis_set_.Size();
}

void Inspection::Graph::Save(const String file_name, const bool save_configs, const Idx dof) const {
	// write in a common format for all robots
	String vertex_file = file_name + "_vertex";
	String edge_file = file_name + "_edge";
	String config_file = file_name + "_conf";
	std::ofstream fout;

	fout.open(vertex_file);
	if (!fout.is_open()) {
		std::cerr << "Vertex file cannot be opened!" << std::endl;
		exit(1);
	}

	for (auto && v : vertices_) {
		fout << v->index << " "
		<< v->time_vis << " "
		<< v->time_build << " ";

		for (Idx t = 0; t < MAX_COVERAGE_SIZE; ++t) {
			if (v->vis[t]) {
				fout << t << " ";
			}
		}
		fout << std::endl;
	}
	fout.close();
	std::cout << "Vertices saved!" << std::endl;

	fout.open(edge_file);
	if (!fout.is_open()) {
		std::cerr << "Edge file cannot be opened!" << std::endl;
		exit(1);
	}

	for (auto && e : edges_) {
		fout << e->source << " "
		<< e->target << " "
		<< e->checked << " "
		<< e->valid << " "
		<< e->time_forward_kinematics << " "
		<< e->time_collision_detection << " "
		<< e->cost << " "
		<< std::endl;
	}
	fout.close();
	std::cout << "Egdes saved!" << std::endl;

	if (!save_configs) {
		return;
	}

	fout.open(config_file);
	if (!fout.is_open()) {
		std::cerr << "Configuration file cannot be opened!" << std::endl;
		exit(1);
	}

#if USE_CRISP
	for (auto && v : vertices_) {
		const CrispStateSpace::StateType *s = v->state->as<CrispStateSpace::StateType>();
		
		fout << v->index << " "
		<< s->Insertion(0) << " "
		<< s->Insertion(1) << " "
		<< s->Quaternion(0).w() << " "
		<< s->Quaternion(0).x() << " "
		<< s->Quaternion(0).y() << " "
		<< s->Quaternion(0).z() << " "
		<< s->Quaternion(1).w() << " "
		<< s->Quaternion(1).x() << " "
		<< s->Quaternion(1).y() << " "
		<< s->Quaternion(1).z() << " "
		<< s->TipTranslation()(0) << " "
		<< s->TipTranslation()(1) << " "
		<< s->TipTranslation()(2) << " "
		<< s->TipTangent()(0) << " "
		<< s->TipTangent()(1) << " "
		<< s->TipTangent()(2) << " ";

		auto vec = s->KinStateVector();
		for (Idx i = 0; i < AUGMENTED_DIMENSION; ++i) {
			fout << vec(i) << " ";
		}
		fout << s->Stability()
		<< std::endl;
	}
#else

#if USE_PLANAR
	// Planar robot.
	for (auto && v : vertices_) {
		const auto s = v->state->as<ob::RealVectorStateSpace::StateType>();

		fout << v->index << " ";
		for (Idx i = 0; i < dof; ++i) {
            fout << s->values[i] << " ";
        }

        fout << std::endl;
	}

#else
	// Drone robot.
	for (auto && v : vertices_) {
		const auto s = v->state->as<DroneStateSpace::StateType>();

		fout << s->Position().transpose() << " "
		<< s->Yaw() << " "
		<< s->CameraAngle() << std::endl;
	}

#endif

#endif
	fout.close();
	std::cout << "Configurations saved!" << std::endl;

}

void Inspection::Graph::ReadFromFiles(const String file_name, const bool read_configs, const Idx dof) {
	String vertex_file = file_name + "_vertex";
	String edge_file = file_name + "_edge";
	String config_file = file_name + "_conf";

	std::ifstream fin;

	fin.open(vertex_file);
	if (!fin.is_open()) {
		std::cerr << "Vertex file cannot be opened!" << std::endl;
		exit(1);
	}

	String line;
	Idx i = 0;
	while (getline(fin, line)) {
		std::istringstream sin(line);
		String field;
		Idx j = 0;
		while(getline(sin, field, ' ')) {
			if (j == 0) {
				if (i != std::stoi(field)) {
					std::cerr << "Incorrect vertex index!" << std::endl;
					exit(1);
				}
				AddVertex(i);
			}
			else if (j == 1) {
				vertices_[i]->time_vis = std::stoi(field);
			}
			else if (j == 2) {
				vertices_[i]->time_build = std::stoi(field);
			}
			else {
				vertices_[i]->vis.Insert(std::stoi(field));
			}
			j++;
		}
		UpdateGlobalVisibility(vertices_[i]->vis);
		i++;
	}
	fin.close();
	std::cout << "Vertices read!" << std::endl;

	fin.open(edge_file);
	if (!fin.is_open()) {
		std::cerr << "Edge file cannot be opened!" << std::endl;
		exit(1);
	}

	while (getline(fin, line)) {
		std::istringstream sin(line);
		
		Idx source, target;
		sin >> source >> target;
		EPtr edge(new Inspection::Edge(source, target));

		sin >> edge->checked
		>> edge->valid
		>> edge->time_forward_kinematics
		>> edge->time_collision_detection
		>> edge->cost;

		AddEdge(edge);
	}
	fin.close();
	std::cout << "Edges read!" << std::endl;

	if (!read_configs) {
		return;
	}

	fin.open(config_file);
	if (!fin.is_open()) {
		std::cerr << "Configuration file cannot be opened!" << std::endl;
		exit(1);
	}

#if USE_CRISP
	i = 0;
	auto space = std::make_shared<CrispStateSpace>(2);
    while (getline(fin, line)) {
        std::istringstream sin(line);
        String field;
        Idx index;
        RealNum ins0, ins1, stability;
        Quat q0, q1;
        Vec3 trans, tang;
        crisp::KinVec vec;

        sin >> index;
        if (i != index) {
            std::cerr << "Incorrect vertex index!" << std::endl;
            exit(1);
        }

        sin >> ins0 >> ins1
        >> q0.w() >> q0.x() >> q0.y() >> q0.z()
        >> q1.w() >> q1.x() >> q1.y() >> q1.z()
        >> trans[0] >> trans[1] >> trans[2]
        >> tang[0] >> tang[1] >> tang[2];

        for (Idx j = 0; j < AUGMENTED_DIMENSION; ++j) {
            sin >> vec(j);
        }
        sin >> stability;

        vertices_[i]->state = space->allocState();
        CrispStateSpace::StateType *s = vertices_[i]->state->as<CrispStateSpace::StateType>();
        s->SetInsertion(0, ins0);
        s->SetInsertion(1, ins1);
        s->SetQuaternion(0, q0);
        s->SetQuaternion(1, q1);
        s->SetTipTranslation(trans);
        s->SetTipTangent(tang);
        s->SetKinStateVector(vec);
        s->SetStability(stability);

        i++;
    }
#else
	// Planar robot.
    i = 0;
    auto space = ob::StateSpacePtr(new ob::RealVectorStateSpace(dof));
     while (getline(fin, line)) {
        std::istringstream sin(line);
        String field;
        Idx index;
        std::vector<RealNum> config(dof, 0.0);

        sin >> index;
        if (i != index) {
            std::cerr << "Incorrect vertex index!" << std::endl;
            exit(1);
        }

        for (Idx i = 0; i < dof; ++i) {
            sin >> config[i];
        }

        vertices_[i]->state = space->allocState();
        ob::RealVectorStateSpace::StateType *s = vertices_[i]->state->as<ob::RealVectorStateSpace::StateType>();
        for (Idx i = 0; i < dof; ++i) {
            s->values[i] = config[i];
        }

        i++;
    }

#endif
	fin.close();
	std::cout << "Configurations read!" << std::endl;


}



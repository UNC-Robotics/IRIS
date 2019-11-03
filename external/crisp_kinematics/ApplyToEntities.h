//
//  ApplyToEntities.h
//  SnareNeedleModel
//
//  Created by Art Mahoney on 7/21/16.
//  Copyright © 2016 Art Mahoney. All rights reserved.
//

#ifndef ApplyToEntities_h
#define ApplyToEntities_h

#include <typeinfo>

class BunchBase;


//class NodeBase;
//class BunchBase {};


namespace Helper {
    template <int... Is>
    struct index {};
    
    template <int N, int... Is>
    struct gen_seq : gen_seq<N - 1, N - 1, Is...> {};
    
    template <int... Is>
    struct gen_seq<0, Is...> : index<Is...> {};
}



template<class FunctorClass, class ParentType, class ...Ts>
class Apply {
public:
    Apply(FunctorClass &functor, const ParentType &parent, Ts... ts) {};
};


template<class FunctorClass, class ParentType, class T, class ...Ts>
class Apply<FunctorClass, ParentType, T, Ts...> : public Apply<FunctorClass, ParentType, Ts...> {
private:
    // Not a bunch entity.
    
    template<class T2, typename std::enable_if<!std::is_base_of<BunchBase, T2>::value, int>::type = 0>
    void applyToEntity(FunctorClass &functor, const ParentType &parent, T2 &t) {
        functor(t, parent); // apply the functor to this node
        t.template applyFunctor<FunctorClass, T2>(functor, t); // expand this nodes children
    }
    
    
    // This is a bunch entity.
    template<class T2, typename std::enable_if<std::is_base_of<BunchBase, T2>::value, int>::type = 0>
    void applyToEntity(FunctorClass &functor, const ParentType &parent, T2 &t) {
        t.template applyFunctor<FunctorClass, ParentType>(functor, parent); // expand this nodes children, but don't apply the functor to a bunch itself (only its children)
    }
    
public:
    
    Apply(FunctorClass &functor, const ParentType &parent, T t, Ts... ts) : Apply<FunctorClass, ParentType, Ts...>(functor, parent, ts...) {
        //functor(t, parent); // apply the functor to this node
        //t.template applyFunctor<FunctorClass, T>(functor, t); // expand this nodes children
        
        applyToEntity(functor, parent, t);
        
    }
    
};





template<class FunctorClass, class ...Ts>
class ApplyToList {
public:
    ApplyToList(FunctorClass &functor, Ts... ts) {};
};


template<class FunctorClass, class T, class ...Ts>
class ApplyToList<FunctorClass, T, Ts...> : public ApplyToList<FunctorClass, Ts...> {
private:
    
    
    

    
    /**
     * Apply the functor to an entity that isn't a bunch entity.
     *
     */
    template<class T2, typename std::enable_if<!std::is_base_of<BunchBase, T2>::value, int>::type = 0>
    void applyToEntity(FunctorClass &functor, T2 &t) {

        

        //std::cout << "applied to entity " << std::endl;
        //std::cout << std::is_base_of<BunchBase,T2>::value << " " << typeid(t).name() << std::endl;
        functor(t);
    }
    
    template<class T2, typename std::enable_if<std::is_base_of<BunchBase,T2>::value, int>::type = 0>
    void applyToEntity(FunctorClass &functor, T2 &t) {

        //std::cout << "applied to a bunch" << std::endl;
        
        t.applyToList(functor);

        return;
    }
    
    

    


    
    

public:
    ApplyToList(FunctorClass &functor, T t, Ts... ts) : ApplyToList<FunctorClass, Ts...>(functor, ts...) {
        
        applyToEntity<T>(functor, t);
    }
    
};




template<class FunctorClass, class ...Ts>
struct ApplyToListReverse {
    template<class ...Us>
    static void apply(FunctorClass &functor, Us... us) {}
};


template<class FunctorClass, class T, class ...Ts>
struct ApplyToListReverse<FunctorClass, T, Ts...>  {
    template<class U, class ...Us>
    static void apply(FunctorClass &functor, U u, Us... us) {
        functor(u);
        ApplyToListReverse<FunctorClass, Ts...>::apply(functor, us...);
    }
};


template<class FunctorClass, class T, class ...Ts>
struct ApplyToListInOrder {
    ApplyToListInOrder(FunctorClass &functor, T t, Ts... ts) {
        ApplyToListReverse<FunctorClass, T, Ts...>::apply(functor, t, ts...);
    }
};











#endif /* ApplyToEntities_h */

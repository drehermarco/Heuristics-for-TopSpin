#ifndef TOPSPINSTATESPACE_H
#define TOPSPINSTATESPACE_H

#include <iostream>
#include <vector>
using namespace std;

class TopSpinStateSpace {
public:
    class TopSpinState {
    public:
        vector<int> permutation;
        int size;
        TopSpinState();
        TopSpinState(const vector<int>& perm);
        bool operator==(const TopSpinState& other) const;
        friend ostream& operator<<(ostream& os, const TopSpinState& state);
    };

    class TopSpinAction {
    public:
        int rotate;
        TopSpinAction(int r);
        int cost() const;
        void apply(TopSpinState& state) const;
        friend ostream& operator<<(ostream& os, const TopSpinAction& action);
    };

    class TopSpinActionStatePair {
    public:
        TopSpinAction action;
        TopSpinState state;
        TopSpinActionStatePair(const TopSpinAction& a, const TopSpinState& s);
    };

    int n;
    TopSpinState initialState;
    vector<TopSpinAction> actions;

    TopSpinStateSpace(int size, TopSpinState initial);
    TopSpinState getInitialState() const;
    bool is_Goal(const TopSpinState& state) const;
    vector<TopSpinActionStatePair> successors(const TopSpinState& state) const;
    int h(const TopSpinState& state) const;
};

#endif

#ifndef TOPSPINSTATESPACE_H
#define TOPSPINSTATESPACE_H

#include <iostream>
#include <vector>

class TopSpinStateSpace {
public:
    class TopSpinState {
    public:
        std::vector<int> permutation;
        int size;
        int k;
        TopSpinState();
        TopSpinState(const std::vector<int>& perm);
        bool operator==(const TopSpinState& other) const;
        friend std::ostream& operator<<(std::ostream& os, const TopSpinState& state);
    };

    class TopSpinAction {
    public:
        int rotate;
        TopSpinAction(int r);
        int cost() const;
        void apply(TopSpinState& state) const;
        friend std::ostream& operator<<(std::ostream& os, const TopSpinAction& action);
    };

    class TopSpinActionStatePair {
    public:
        TopSpinAction action;
        TopSpinState state;
        TopSpinActionStatePair(const TopSpinAction& a, const TopSpinState& s);
    };

    int n;
    TopSpinState initialState;
    std::vector<TopSpinAction> actions;

    TopSpinStateSpace(int size, TopSpinState initial);
    TopSpinState getInitialState() const;
    bool is_Goal(const TopSpinState& state) const;
    std::vector<TopSpinActionStatePair> successors(const TopSpinState& state) const;
    int h(const TopSpinState& state) const;
};

#endif

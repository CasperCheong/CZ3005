/* predicates */
male(prince_charles) .
male(prince_andrew) . 
male(prince_edward) .

female(queen_elizabeth) .
female(princess_ann) .

parent(queen_elizabeth, prince_charles) .
parent(queen_elizabeth, prince_andrew) .
parent(queen_elizabeth, prince_edward) .
parent(queen_elizabeth, princess_ann) .


older(prince_charles, princess_ann) .
older(princess_ann, prince_andrew) .
older(prince_andrew, prince_edward) .

/* rules */

is_older(X, Y) :- older(X, Y) .
is_older(X, Y) :- older(X, Z), is_older(Z, Y) .

son(X, Y) :- parent(Y,X), male(X) .
daughter(X, Y) :- parent(Y,X), female(X) .

succession_rule(X,Y) :- parent(Z, X), parent(Z, Y), 
                        is_older(X, Y),
                        Y\=queen_elizabeth .
                          

%% Sorting algorithm
succession_sort([A|B], Sorted) :- succession_sort(B, SortedTail), insert(A, SortedTail, Sorted).
succession_sort([], []).

insert(A, [B|C], [B|D]) :- not(succession_rule(A,B)), !, insert(A, C, D).
insert(A, C, [A|C]).

royalSuccessionList(X, SuccessionList):-
	findall(Y, parent(X,Y), Children),
	succession_sort(Children, SuccessionList).
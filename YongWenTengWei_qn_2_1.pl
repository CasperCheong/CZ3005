/* predicates */
male(prince_charles) .
male(prince_andrew) . 
male(prince_edward) .

female(queen_elizabeth) .
female(princess_ann) .

offspring(prince_charles, queen_elizabeth) .
offspring(princess_ann, queen_elizabeth) .
offspring(prince_andrew, queen_elizabeth) .
offspring(prince_edward, queen_elizabeth) .


older(prince_charles, princess_ann) .
older(princess_ann, prince_andrew) .
older(prince_andrew, prince_edward) .

/* rules */
is_older(X, Y) :- older(X, Y) .
is_older(X, Y) :- older(X, Z), is_older(Z, Y) .

son(X,Y) :- offspring(X,Y), male(X) .
daughter(X,Y) :- offspring(X,Y), female(X) .

%% Succession Rule 1: Male offspring will always precede female offspring
succession_rule(X,Y) :- son(X, Z), daughter(Y,Z),
                        Y\=queen_elizabeth .

%% Succession Rule 2: The elder male offspring will always precede younger male offspring                         
succession_rule(X,Y) :- son(X, Z), son(Y, Z), is_older(X, Y) .


%% Succession Rule 3: The elder female offspring will always precede younger female offspring
succession_rule(X,Y) :- daughter(X, Z), daughter(Y, Z), is_older(X, Y),
                        X\=queen_elizabeth, Y\=queen_elizabeth .


%% Sorting algorithm
succession_sort([A|B], Sorted) :- succession_sort(B, SortedTail), insert(A, SortedTail, Sorted).
succession_sort([], []).

insert(A, [B|C], [B|D]) :- not(succession_rule(A,B)), !, insert(A, C, D).
insert(A, C, [A|C]).

royalSuccessionList(X, SuccessionList):-
	findall(Y, offspring(Y, X), Children),
	succession_sort(Children, SuccessionList).
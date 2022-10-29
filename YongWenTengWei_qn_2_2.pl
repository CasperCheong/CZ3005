child(charles) .
child(ann) .
child(andrew) .
child(edward) . 

parent(elizabeth, charles) .
parent(elizabeth, ann) .
parent(elizabeth, andrew) .
parent(elizabeth, edward) .


successor(X, Y) :- parent(X, Y) .

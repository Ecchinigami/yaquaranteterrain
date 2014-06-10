quicksort([],[]).
 
quicksort([X|Tail],Sorted):-
 split(X,Tail,Small,Big),
 quicksort(Small,SortedSmall),
 quicksort(Big,SortedBig),
 conc(SortedSmall, [X|SortedBig],Sorted).
 
split(X,[],[],[]).
 
split(X,[Y|Tail],[Y|Small],Big):-
 gt(X,Y),!,
 split(X,Tail,Small,Big).
 
split(X,[Y|Tail],Small,[Y|Big]):-
 split(X,Tail,Small,Big).
 
gt([X_numero_noeud, X_parent_noeud, X_pos_courante, X_cout],[Y_numero_noeud, Y_parent_noeud, Y_pos_courante, Y_cout]): X_cout > Y_cout.
 
conc([],L,L).
conc([X|L1],L2,[X|L3]):-
 conc(L1,L2,L3).

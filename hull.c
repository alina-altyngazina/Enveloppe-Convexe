
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>

struct vec{
    double x;
    double y;
};

struct vecset {
    struct vec *data;
    size_t size;
    size_t capacity;
};

#define BUFSIZE 1024

double dot(const struct vec *v1, const struct vec *v2);
double cross(const struct vec *p1,const struct vec *p2, const struct vec *p3);
bool is_left_turn(const struct vec *p1,const struct vec *p2, const struct vec *p3);
void vecset_create(struct vecset *self);
void vecset_destroy(struct vecset *self);
void vecset_add(struct vecset *self, struct vec p);
typedef int (*comp_func_t)(const struct vec *p1,const struct vec *p2, const void *ctx);
int comp_x(const struct vec *p1,const struct vec *p2, const void *ctx);
int comp_y(const struct vec *p1,const struct vec *p2, const void *ctx);
int comp_atan(const struct vec *p1,const struct vec *p2, const void *ctx);
const struct vec *vecset_max(const struct vecset *self,comp_func_t func, const void *ctx);
const struct vec *vecset_min(const struct vecset *self,comp_func_t func, const void *ctx);
void vecset_sort(struct vecset *self, comp_func_t func,const void *ctx);
void vecset_push(struct vecset *self, struct vec p);
void vecset_pop(struct vecset *self);
const struct vec *vecset_top(const struct vecset *self);
const struct vec *vecset_second(const struct vecset *self);
void jarvis_march(const struct vecset *in, struct vecset *out);
void graham_scan(const struct vecset *in, struct vecset *out);
void quickhull(const struct vecset *in, struct vecset *out);
struct vecset *findhull(const struct vecset *s,const struct vec *x,const struct vec *y);

int main() {
    struct vecset *in=malloc(sizeof(struct vecset));
    struct vecset *out=malloc(sizeof(struct vecset));
    vecset_create(in);
    vecset_create(out);
    setbuf(stdout, NULL); 
    // avoid buffering in the output
    char buffer[BUFSIZE];
    fgets(buffer, BUFSIZE, stdin);
    size_t count = strtol(buffer, NULL, 10);
    for (size_t i = 0; i < count; ++i) {
        struct vec p;
        fgets(buffer, BUFSIZE, stdin);
        char *endptr = buffer;
        p.x = strtod(endptr, &endptr);
        p.y = strtod(endptr, &endptr);
        vecset_push(in,p);
    }
    
    // jarvis_march(in,out);
    graham_scan(in,out);
    // quickhull(in,out);

    //Envoie de l'ensemble des points (out) de l'enveloppe convexe sur stdout et sauvegarder dans le fichier hull.log
    FILE *f;
    f=fopen("hull.log","w");
    fprintf(stdout,"%ld\n",out->size);
    fprintf(f,"%ld\n",out->size);
    for (int i = 0; i < out->size; i++){
        fprintf(stdout,"%f %f\n",out->data[i].x,out->data[i].y);
        fprintf(f,"%f %f\n",out->data[i].x,out->data[i].y);
    }
    fflush(stdout);
    fclose(f);
    free(in->data);
    free(out->data);
    vecset_destroy(in);
    vecset_destroy(out);
    return 0;
}

//  fonction qui calcule le produit  scalaire de V1 et V2
double dot(const struct vec *v1, const struct vec *v2){
    return v1->x*v2->x+v1->y*v2->y;
}

// fonction qui calcule le produit vectoriel 2D de deux vecteurs P1P2 et P1P3
double cross(const struct vec *p1,const struct vec *p2, const struct vec *p3){
    return (p2->x-p1->x)*(p3->y-p1->y)-(p2->y-p1->y)*(p3->x-p1->x);
}

// fonction qui permit de savoir si p3 fait un tournant à gauche par rapport à p1 et p2
bool is_left_turn(const struct vec *p1,const struct vec *p2, const struct vec *p3){
    if(cross(p1,p2,p3)<0.){
        return true;
    }
    else{
        return false;
    }
}

// fonction qui crée un ensemble de points vide avec une capacité de 10
void vecset_create(struct vecset *self){
    self->data=calloc(10,sizeof(struct vec));;
    self->size=0;
    self->capacity=10;
}

// fonction qui détruit un ensemble de points.
void vecset_destroy(struct vecset *self){
    free(self);
}

// fonction qui ajoute un point à un ensemble de points
void vecset_add(struct vecset *self, struct vec p){
    self->data[self->size]=p;
    self->size++;
}

// fonction qui compare deux points p1 et p2 par rapport a reference ctx et par rapport a l'axe y
    // si les coordonnees de point p1 > p2, renvoie 1;
        // si p1<p2 renvoie -1;
            // si p1=p2 renvoie 0;
                // vue que ctx est un pointeur de type void, on doit definir son type *(double*);
int comp_y(const struct vec *p1,const struct vec *p2, const void *ctx){
    if(p1->y-(*(double*)ctx+1)>p2->y-(*(double*)ctx+1)){
        return 1;
    }
    else if(p1->y-(*(double*)ctx+1)<p2->y-(*(double*)ctx+1)){
        return -1;
    }
    else{
        return 0;
    }
}

// fonction qui compare deux points p1 et p2 par rapport a reference ctx et par rapport a l'axe x
    // si les coordonnees de point p1 > p2, renvoie 1;
        // si p1<p2 renvoie -1;
            // si p1=p2 renvoie 0;
                // vue que ctx est un pointeur de type void, on doit definir son type *(double*);
int comp_x(const struct vec *p1,const struct vec *p2, const void *ctx){
    if(p1->x-*(double*)ctx>p2->x-*(double*)ctx){
        return 1;
    }
    else if(p1->x-*(double*)ctx<p2->x-*(double*)ctx){
        return -1;
    }
    else{
        return 0;
    }
}

// on compare les deux points p.r. a l'angle(atan2) que chaque point fais avec ctx(le point b) par ordre croissant
int comp_atan(const struct vec *p1,const struct vec *p2, const void *ctx){
    double atan_p1=atan2(p1->y-(*((double *)ctx+1)),p1->x-(*(double *)ctx));
    double atan_p2=atan2(p2->y-(*((double *)ctx+1)),p2->x-(*(double *)ctx));
    if(atan_p1>atan_p2){
        return 1;
    }
    else if(atan_p1<atan_p2){
        return -1;
    }
    else{
        return 0;
    }
}

// On calcule le point max dans un ensemble, en utilisant la fonction comp_func_t func;
    // On parcours l'ensemble et on compare chaque element avec tous les autres 
        // a chaque fois qu'on trouve un element > que l'autre, on incremant une variable k;
            // et quand cette variable est egale a la taille, on renvoie l'element de l'indice i;
const struct vec *vecset_max(const struct vecset *self,comp_func_t func, const void *ctx){
    int k=0;
    for(int i=0;i<=self->size-1;i++){
        for(int j=0;j<=self->size-1;j++){
            if(func(&self->data[i],&self->data[j],ctx)==1){
                k++;
            }
            if(k==self->size-1){
                return &self->data[i];
            }
        }
        k=0;
    }
}

// On calcule le point min dans un ensemble, en utilisant la fonction comp_func_t func;
    // On parcours l'ensemble et on compare chaque element avec tous les autres 
        // a chaque fois qu'on trouve un element < que l'autre, on incremant une variable k;
            // et quand cette variable est egale a la taille, on renvoie l'element de l'indice i;
const struct vec *vecset_min(const struct vecset *self,comp_func_t func, const void *ctx){
    int k=0;
    for(int i=0;i<=self->size-1;i++){
        for(int j=0;j<=self->size-1;j++){
            if(func(&self->data[i],&self->data[j],ctx)==-1){
                k++;
            }
            if(k==self->size-1){
                return &self->data[i];
            }
        }
        k=0;
    }
}

//  On organise l'ensemble des points p.r. a la fonction de comparaison donnee
//  par ordre croissant;
void vecset_sort(struct vecset *self, comp_func_t func,const void *ctx){
    struct vec var;
    for(int j=0;j<self->size;j++){
        for(int i=j+1;i<self->size;i++){
            if(func(&self->data[j],&self->data[i],ctx)==1){
                var=self->data[j];
                self->data[j]=self->data[i];
                self->data[i]=var;
            }
        }
    }
}

// On rajoute un element a un ensemble;
    // Quand le pile est complet, on augmente la taille;
        // On double la taille;
void vecset_push(struct vecset *self, struct vec p){
    if(self->capacity<=self->size){
        self->capacity=self->capacity*2;
        struct vec *a=calloc(self->capacity,sizeof(struct vec));
        a=memcpy(a,self->data,self->size*sizeof(struct vec));
        free(self->data);
        self->data=a;
    }
    vecset_add(self,p);
}

// On enleve le premier element de la pile;
void vecset_pop(struct vecset *self){
    if(self->size>0){
        self->data[self->size-1].x=0;
        self->data[self->size-1].y=0;
        self->size--;
    }else self->size=0;
}

// Renvoie le premier element de la pile;
const struct vec *vecset_top(const struct vecset *self){
    return &self->data[self->size-1];
}

// Renvoie la deuxieme element de la pile;
const struct vec *vecset_second(const struct vecset *self){
    return &self->data[self->size-2];
}

// Algorithme 1:
void jarvis_march(const struct vecset *in, struct vecset *out){
    struct vec ff; //un vecteur temporaire;
    const struct vec *f; //premier element;
    struct vec *c,*n; //vecteur courant et next;
    double ref[2]={0,0}; //reference par rapport a quoi on fait la comparaison
    comp_func_t func=comp_x; //pointeur de fonction
    f=vecset_min(in,func,ref); //calcule le min par rapport a l'axe x
    c=(void *)f; //pour sauvegarder f(const) a c(non const), on utilise (void*) pour enlever const dans f
    int k=0; //pour parcourir l'ensemble des elements de in
    do{
        ff.x=c->x; //sauvegarde  les coordonnes de c(pointeur) dans ff(vec temporaire)
        ff.y=c->y; //pour pouvoir l'utiliser dans un fonction vecset_push (qui prend pas des pointeurs)
        if(f==&in->data[k]) { //si f (min par rapport a l'axe x) == a 1er element de in, on prend l'element suivant
            k++;
        }
        n=&in->data[k]; //n est un point de in
        vecset_push(out,ff); //on sauvegarde le point min dans out pendant la 1er iteration, apres ff devient le point courant
        if(vecset_top(out)->x==vecset_second(out)->x){  //on verifie si le premier est le second sont =, si oui enleve le premier
            vecset_pop(out);
        }
        for(int i=0;i<in->size;i++){  //on teste le point courant c avec les points suivants n, avec tout les points  
            if (is_left_turn(c,&in->data[i],n)==true){ //pour voir s'ils font un tournant a gauche et on sauvegarde dans n; 
                n=&in->data[i];
            }
        }
        c=n; //point courant = a point suivant, pour la 2eme iteration
        if(k==in->size-1)k=-1;  //si k = la taille de in, on le remis a 0;
        k++;
    }while(c!=f);  //on continue la boucle tq point courrent != de point first(1er element de l'enveloppe)
    if(out->size>in->size){  //on verifie si la taille de out depasse pas la taille de in. Si oui, enleve le premier element
        vecset_pop(out);
    }
}

// Algorithme 2:

void graham_scan(const struct vecset *in, struct vecset *out){
    struct vec f,ff; //vecteurs temporarires 
    const struct vec *b; //le point plus petit sur ordonnee
    const struct vec *t=NULL,*s=NULL; //t est premier element, s le second de out
    double ref[2]={0,0};  //reference par rapport a quoi on fait la comparaison
    comp_func_t func=comp_y; //pointeur de fonction
    struct vecset *tmp; //l'ensemble de point temporarire pour le trie de in;
    tmp=(void *)in;
    b=vecset_min(tmp,func,ref);  //calcule le min par rapport a l'axe x
    for(int i=0;i<tmp->size;i++){  //en cas d’égalité, on choisi le point de plus petite abscisse
        if(tmp->data[i].y==b->y){
            if(tmp->data[i].x<b->x){
                b=&tmp->data[i];
            }   
        }
    }
    ff.x=b->x;
    ff.y=b->y;
    double ref1[2]={ff.x,ff.y};
    comp_func_t func2=comp_atan;
    vecset_sort(tmp,func2,ref1);  //on trie l'ensemble tmp du plus petit angle au plus grand p.r. a ctx=b;
    f=tmp->data[1];  //1er element dans tmp different de b. (b=tmp->data[0])
    vecset_push(out,ff);  //on sauvegarde b et f dans out
    vecset_push(out,f);
    for(int i=2;i<tmp->size;i++){  //on parcours les element a partir de l'indice 2
        if(&tmp->data[i]!=&ff && &tmp->data[i]!=&f){  //tq les element son differents de b et f;
            t=vecset_top(out);
            s=vecset_second(out);
            if(out->size>=2 && is_left_turn(s,t,&tmp->data[i])==true){
                vecset_pop(out);
                i--;    //Decrementation pour pouvoir retester le meme element apres avoir fait vecset_pop;
            }else vecset_push(out,tmp->data[i]);  //on mis dans out le nouvel point a tester dans la prochaine iteration de la boucle
        }
    }
}

/*Algoritme 3:
    On a pas reussi a tout implementer dans cette methode.
    Il manque toujours des points dans le calcul de l'enveloppe convexe. 
*/
void quickhull(const struct vecset *in, struct vecset *out){
    const struct vec *a,*b;
    struct vec aa,bb;
    struct vecset *s1,*s2;
    struct vecset *r1,*r2;
    s1=malloc(sizeof(struct vecset));
    vecset_create(s1);
    s2=malloc(sizeof(struct vecset));
    vecset_create(s2);
    r1=malloc(sizeof(struct vecset));
    vecset_create(r1);
    r2=malloc(sizeof(struct vecset));
    vecset_create(r2);
    double ref[2]={0,0};
    comp_func_t func=comp_x;
    a=vecset_min(in,func,ref);
    b=vecset_max(in,func,ref);
    for(int i=0;i<in->size;i++){
        if(&in->data[i]!=a || &in->data[i]!=b){
            if(is_left_turn(a,b,&in->data[i])==true){
                vecset_push(s1,in->data[i]);
            }
            else{
                vecset_push(s2,in->data[i]);
            }
        }
    }
    r1=findhull(s1,a,b);
    r2=findhull(s2,b,a);
    free(s1->data);
    free(s2->data);
    vecset_destroy(s1);
    vecset_destroy(s2);
    aa.x=a->x;
    aa.y=a->y;
    vecset_push(out,aa);
    for(int i=0;i<r1->size;i++){
        vecset_push(out,r1->data[i]);
    }
    bb.x=b->x;
    bb.y=b->y;
    vecset_push(out,bb);
    for(int i=0;i<r2->size;i++){
        vecset_push(out,r2->data[i]);
    }
    free(r1->data);
    free(r2->data);
    vecset_destroy(r1);
    vecset_destroy(r2);
}

struct vecset *findhull(const struct vecset *s,const struct vec *x,const struct vec *y){
    struct vecset *out;
    struct vec *m;
    struct vec mm;
    struct vecset *r1,*r2;
    out=malloc(sizeof(struct vecset));
    vecset_create(out);
    r1=malloc(sizeof(struct vecset));
    vecset_create(r1);
    r2=malloc(sizeof(struct vecset));
    vecset_create(r2);
    if(s->size==0){
        return NULL;
    }else{
    struct vec *a,*b;
    struct vecset *m1;
    struct vecset *s1,*s2;
    
    s1=malloc(sizeof(struct vecset));
    vecset_create(s1);
    s2=malloc(sizeof(struct vecset));
    vecset_create(s2);
    
    /*
        Pour calculer le point le plus eloignee de XY, on a cherche le centre de segment forme par XY
            Et on le passe comme un ctx dans les fonctions de comparaisons.
    */
    double a1=0.,b1=0.;
    double ref[2]={(x->x-y->x)/2,(x->y-y->y)/2};
    comp_func_t func=comp_y;
    a=(void *)vecset_min(s,func,ref);
    b=(void *)vecset_max(s,func,ref);
    if(a->y<0 ){
        a1=a->y*(-1);
    }
    else if(b->y<0){
        b1=b->y*(-1);
    }
    if(a1>b1){
        m=a;
    }
    else{
        m=b;
    }
    for(int i=0;i<s->size;i++){
        if(&s->data[i]!=m){
            if(is_left_turn(x,m,&s->data[i])==true){
                vecset_push(s1,s->data[i]);
            }
            else if(is_left_turn(m,y,&s->data[i])==true){
                vecset_push(s2,s->data[i]);
            }
        }
    }

    // mm.x=m->x;
    // mm.y=m->y;
    // vecset_push(r1,mm);
    findhull(s1,x,m);
    // mm.x=m->x;
    // mm.y=m->y;
    // vecset_push(r2,mm);
    findhull(s2,m,y);
    
    free(s1->data);
    free(s2->data);
    vecset_destroy(s1);
    vecset_destroy(s2);
    }
    for(int i=0;i<r1->size;i++){
         vecset_push(out,r1->data[i]);
    }
    mm.x=m->x;
    mm.y=m->y;
    vecset_push(out,mm);
    for(int i=0;i<r2->size;i++){
         vecset_push(out,r2->data[i]);
    }
    free(r1->data);
    free(r2->data);
    vecset_destroy(r1);
    vecset_destroy(r2);
    
    return out;
}
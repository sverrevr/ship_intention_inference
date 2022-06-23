#ifndef SMILE_GENERALCLASES_H
#define SMILE_GENERALCLASES_H

// {{SMILE_PUBLIC_HEADER}}

#include "stringarray.h"
#include "idarray.h"
#include <vector>

class DSL_node;


class DSL_header : public DSL_object
{
public:
	DSL_header();
	DSL_header(DSL_header &likeThisOne);
	~DSL_header();
	int operator =(const DSL_header &likeThisOne);
	void CleanUp(int deep = 0);
	int SetId(const char *theID);
	int SetName(const char *theName);
	int SetComment(const char *theComment);
	const char *GetId() const { return id; }
	const char *GetName() const { return name; }
	const char *GetComment() const { return comment; }
	static int IsThisIdValid(const char *id);
	static int MakeValidId(char *id);
	void CheckConsistency(int deep = 1);
	void AssociateWithNode(DSL_node *n);
private:
	const char *id;       // For quick reference (including formulae), no spaces.
	const char *name;     // Longer name for display and printing, spaces allowed.
	const char *comment;  // A possibly longer text
	DSL_node *node; // node associated with this header (may be NULL for network/submodel headers)
};

class DSL_rectangle
{
public:
    DSL_rectangle() { center_X = center_Y = width = height = 0; }
    int FillDefaultValues(const DSL_rectangle &fromHere);
    void Set(int cX, int cY, int W, int H) { center_X = cX; center_Y = cY, width = W; height = H; }
    int center_X;
    int center_Y;
    int width;
    int height;
};



class DSL_screenInfo
{
public:
    DSL_screenInfo();
    int FillDefaultValues(const DSL_screenInfo &fromHere);

    DSL_rectangle position;
    int color;              
    int selColor;           
    int font;               
    int fontColor;
    int borderThickness;
    int borderColor;
	bool barchartActive;
	int barchartWidth;
	int barchartHeight;
};


// DSL_textBoxList is just a typedef
typedef std::vector<std::pair<std::string, DSL_rectangle> > DSL_textBoxList;


class DSL_userProperties : public DSL_object
{
 private:
  // NOTE: a user property is a pair ([id],[string])
  DSL_idArray names;
  DSL_stringArray values;

 public:
  DSL_userProperties();
  DSL_userProperties(const DSL_userProperties &likeThisOne);
   int operator =(const DSL_userProperties &likeThisOne);
  void CleanUp(int deep = 0);

  int AddProperty(const char *propertyName, const char *propertyValue);
  int InsertProperty(int here, const char *propertyName, const char *propertyValue);
  const char *GetPropertyName(int index) const {return(names[index]);}; // no checking !!!
  const char *GetPropertyValue(int index) const {return(values[index]);}; // no checking !!!
  int ChangePropertyName(int thisOne, const char *thisName);
  int ChangePropertyValue(int thisOne, const char *thisValue);

  int FindProperty(const char *withThisName) const;
  int DeleteProperty(int thisOne);
  int DeleteAllProperties();
  int GetNumberOfProperties() const ;

  void CheckConsistency(int deep = 1);
};


class DSL_documentation : public DSL_object
{
 // this class represents the documentation part on another
 // component like, for example, a state of a node.
 private:
  // NOTE: a document is a pair ([title],[path])
  DSL_stringArray titles;
  DSL_stringArray paths;

 public:
  DSL_documentation(void);
  DSL_documentation(const DSL_documentation &likeThisOne);
 ~DSL_documentation();
  int operator =(const DSL_documentation &likeThisOne);
  void CleanUp(int deep = 0);

  int AddDocument(const char *title, const char *path);
  int InsertDocument(int here, const char *title, const char *path);
  const char * GetDocumentTitle(int index) const { return titles[index]; }
  const char * GetDocumentPath(int index) const { return paths[index]; }
  int ChangeDocumentTitle(int thisOne, char *newTitle);
  int ChangeDocumentPath(int thisOne, char *newPath);

  int FindDocument(const char *withThisTitle) const;
  int DeleteDocument(int thisOne);
  int DeleteAllDocuments();
  int GetNumberOfDocuments() const;

  void CheckConsistency(int deep = 1);
};

#endif

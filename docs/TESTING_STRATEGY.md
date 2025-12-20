# Testing Strategy

This is a document outlining the testing strategy used for testing the functionality and capabilities of the codebase's systems.

WIP

## .ipp files
Template classes need the full definition of the class in order for programs to fully compile. This is due to how templating works in c++ in general and cannot be avoided. There are several ways of organizing code to meet this requirement, the one we use is using .ipp files. Declaration of the template class is defined in its .hpp file and the implementation is stored in its corresponding .ipp file. The .ipp file is then included at the bottom of the .hpp and effectively extends the header file to include implementation. This is to emulate the normal structure of .hpp and .cpp source files so general documentation can be stored in the .hpp and more specific notes about implementation are included in the .ipp. For more discussion about .ipp files, please see [SYSTEM_DESIGN](./SYSTEM_DESIGN.md).

In regards to testing these template classes (with or without template methods), it is very similar to testing regular classes. While we cannot preform as exhaustive as we normal could due to the template arguments, we can still get good test coverage over the class's behavior under normal circumstances. 

For testing classes which have dependencies on template classes it is more difficult. AFAIK Google Mock does not have the capabilities to easily mock the template methods within template classes, so the only alternative is to use fakes. Essentially, this means implementing the class's functionality, and testing this functionality, twice: one for the real implementation and one for the fake implementation. Additionally, this creates a source of potential error as both real and fake implementations have to follow the design specifications exactly otherwise downstream testing with the fake class will be worthless. This creates a greater cost to having template classes with template methods in the codebase and thus, template classes WITH template methods should be avoided as much as possible unless it is deemed its usefulness outweights the costs.
#import "@preview/finite:0.5.0" as finite
#import finite: automaton
#import "@preview/glossarium:0.5.9": gls, glspl, make-glossary, print-glossary, register-glossary
#show: make-glossary
#import "@preview/circuiteria:0.2.0"
#import "@preview/zebraw:0.6.1": *
// Glossary
#let entry-list = (
  (
    key: "lorme",
    short: "l",
    long: "lorem",
    description: "lorem",
  ),
)

#register-glossary(entry-list)
// Your document body

// Load code snippets reference
#let code = json("./figures/code/index.json")
#let content = yaml("./config/text-content.yaml")

#show raw.where(block: true): zebraw.with(numbering: true)

#let render-snippet(id) = {
  let snippet = code.find(s => s.id == id)
  if snippet == none {
    panic("Snippet not found: " + id)
  }
  show raw: it => it

  zebraw(
    raw(
      read(snippet.path),
      lang: snippet.lang,
      block: true,
    ),
    numbering: true,
    header: [*#snippet.description*],
  )
}
// ============================================
// Main Document - Embedded Lab Documentation
// ============================================



// Document Settings
#set document(
  title: content.document.title,
  author: content.document.authors.map(a => a.name),
  date: auto,
)

// Page Setup
#set page(
  paper: "a4",
  margin: (x: 2.5cm, y: 2.5cm),
  numbering: "1",
  number-align: center,
)

// Text Settings
#set text(
  font: "FreeSerif",
  size: 11pt,
  lang: "de",
)

// Heading Settings
#set heading(numbering: "1.1")

// Paragraph Settings
#set par(justify: true)

// Link styling
#show link: underline

// ============================================
// TITLE PAGE
// ============================================

#align(center)[
  #v(3cm)

  #text(size: 24pt, weight: "bold")[
    #content.title_page.main_title
  ]

  #v(0.5cm)

  #text(size: 18pt)[
    #content.title_page.subtitle
  ]

  #v(2cm)

  #text(size: 14pt)[
    #content.title_page.course
  ]

  #v(1cm)


  #v(1fr)

  // Dynamically generate author information
  #for author in content.document.authors [
    #text(size: 12pt)[
      *#content.title_page.labels.author* #author.name \
      // *#content.title_page.labels.student_id* #author.student_id \
      *#content.title_page.labels.date* #datetime.today().display("[month repr:long] [day], [year]")
    ]
    #v(0.5cm)
  ]


  #v(1fr)
  #text(size: 11pt)[
    #content.title_page.institution.name \
    #content.title_page.institution.department
  ]
]

#pagebreak()

// ============================================
// TABLE OF CONTENTS
// ============================================

#outline(
  title: content.table_of_contents.title,
  indent: auto,
  depth: 3,
)


#pagebreak()
// ============================================
// Main Content
// ============================================

#render-snippet("example-1")


#pagebreak()
// ============================================
// Appendex
// ============================================

= Glossar
#print-glossary(
  entry-list,
)

#pagebreak()

#bibliography(
  "config/bibliography.bib",
  title: "Bibliographie",
  style: "institute-of-electrical-and-electronics-engineers",
)


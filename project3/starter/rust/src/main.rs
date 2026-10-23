use std::{thread, time::Duration};

/// A tiny example of reading a URDF with the vendored roxmltree parser. Real URDFs
/// may carry `<!DOCTYPE robot>`, which roxmltree rejects unless `allow_dtd` is set.
fn print_links(urdf: &str) -> Result<(), roxmltree::Error> {
    let opts = roxmltree::ParsingOptions { allow_dtd: true, ..Default::default() };
    let doc = roxmltree::Document::parse_with_options(urdf, opts)?;
    let robot = doc.root_element();
    for child in robot.children().filter(|n| n.is_element()) {
        if child.tag_name().name() == "link" {
            println!("example link: {}", child.attribute("name").unwrap_or(""));
        }
    }
    Ok(())
}

fn main() {
    let example = r#"<?xml version="1.0"?>
<!DOCTYPE robot>
<robot name="example"><link name="base_link"/><link name="arm"/></robot>"#;
    if let Err(e) = print_links(example) {
        println!("example URDF did not parse: {e}");
    }
    println!("TODO: implement the Project 3 runtime");
    loop {
        thread::sleep(Duration::from_secs(1));
    }
}

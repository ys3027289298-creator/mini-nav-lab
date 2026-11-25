# Count non-blank, non-comment-only lines of C source.
# A line that opens a block comment and closes it on the same line is dropped
# when it has no code left; the state machine keeps multi-line comments from
# inflating a generation's cost. This is a size proxy, not a byte count: what
# the retention decision needs is "how much reader code does this generation
# cost", and comments are free at runtime.
BEGIN { inblk = 0; n = 0 }
{
    line = $0
    out = ""
    i = 1
    while (i <= length(line)) {
        two = substr(line, i, 2)
        if (inblk) {
            if (two == "*/") { inblk = 0; i += 2 } else { i += 1 }
        } else if (two == "/*") {
            inblk = 1; i += 2
        } else if (two == "//") {
            break
        } else {
            out = out substr(line, i, 1); i += 1
        }
    }
    gsub(/[ \t]+$/, "", out)
    gsub(/^[ \t]+/, "", out)
    if (out != "") n++
}
END { print n }
